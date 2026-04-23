package it.unive.pylisa.imports;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PyClassUnit;
import java.io.IOException;
import java.util.Collection;
import org.junit.Test;

/**
 * Exercises the Python-accurate import resolution in
 * {@link it.unive.pylisa.frontend.PythonModuleImportManager}:
 * <ol>
 * <li>When no local {@code json.py} exists, {@code import json} resolves to the
 * stub in {@code src/main/resources/libraries/json.txt}.</li>
 * <li>When a local {@code json.py} sits next to the importer, it shadows the
 * stub — matching {@code sys.path[0]}-first runtime behavior.</li>
 * </ol>
 * Two fingerprints distinguish the resolved path:
 * <ul>
 * <li>The {@link ModuleUnit}'s {@link CodeLocation} — library-spec units carry
 * a {@link SourceCodeLocation} tagged with the {@code location} declared in the
 * {@code .txt} file, whereas units produced by
 * {@code PythonModuleImportManager#loadProjectModule} carry the singleton
 * {@link SyntheticLocation#INSTANCE}.</li>
 * <li>The {@code json.dumps} / {@code json.loads} {@link FunctionUnit} body —
 * the library loader wraps its implementation class in a {@link NativeCFG},
 * while the frontend visitor emits a regular {@code PyCFG} when it parses
 * {@code def dumps(...)} from a real {@code .py} file.</li>
 * </ul>
 */
public class JsonImportResolutionTest {

	@Test
	public void jsonResolvesToStdlibStubWhenNoLocalFile() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/json_stub/main.py",
				false);
		Program program = translator.toLiSAProgram(true);

		assertTrue(
				"Expected the stdlib `json` spec to be loaded when no local json.py exists",
				LibrarySpecificationProvider.isLibraryLoaded("json"));

		ModuleUnit jsonUnit = (ModuleUnit) PyModuleType.lookup("json").getUnit();
		assertNotNull("json ModuleUnit must be registered", jsonUnit);

		// Library-spec units carry a SourceCodeLocation built from `location
		// json`.
		CodeLocation loc = jsonUnit.getLocation();
		assertTrue(
				"Expected json unit to come from the library spec (SourceCodeLocation); got " + loc,
				loc instanceof SourceCodeLocation);
		assertEquals(
				"Library-spec unit should carry the `location json` declared in json.txt",
				"json",
				((SourceCodeLocation) loc).getSourceFile());
		assertFalse(
				"Stub-backed json unit must not carry SyntheticLocation",
				loc == SyntheticLocation.INSTANCE);

		// The two stub methods were registered on the Program under their
		// qualified names during library loading.
		Unit dumpsUnit = program.getUnit("json.dumps");
		assertNotNull("Library-backed json.dumps FunctionUnit must be registered", dumpsUnit);
		assertNotNull(
				"Library-backed json.loads FunctionUnit must be registered",
				program.getUnit("json.loads"));

		// Stub bodies are NativeCFGs wrapping the NoOpFunction implementation
		// declared in json.txt. This is the positive fingerprint that
		// `json.dumps(...)` in main.py is dispatched to the stub, not to a
		// parsed function.
		assertTrue(
				"json.dumps body must be a NativeCFG (stub wrapping NoOpFunction); code members: "
						+ ((FunctionUnit) dumpsUnit).getCodeMembers(),
				((FunctionUnit) dumpsUnit).getCodeMembers().stream().anyMatch(m -> m instanceof NativeCFG));
	}

	@Test
	public void localJsonShadowsStdlibStub() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/json_shadow/main.py",
				false);
		Program program = translator.toLiSAProgram(true);

		// Python-accurate: sys.path[0] beats stdlib. The library spec must NOT
		// have been consulted because `json.py` sits next to main.py.
		assertFalse(
				"Local json.py must shadow the stdlib `json` library spec",
				LibrarySpecificationProvider.isLibraryLoaded("json"));

		ModuleUnit jsonUnit = (ModuleUnit) PyModuleType.lookup("json").getUnit();
		assertNotNull("json ModuleUnit must be registered", jsonUnit);

		// loadProjectModule stamps SyntheticLocation on project-file-backed
		// units — the distinguishing fingerprint vs. the library-spec unit
		// (which would carry a SourceCodeLocation("json", 0, 0)).
		CodeLocation loc = jsonUnit.getLocation();
		assertTrue(
				"Expected json unit to come from loadProjectModule (SyntheticLocation); got " + loc,
				loc == SyntheticLocation.INSTANCE);

		// `def dumps` and `def loads` from json.py become FunctionUnits whose
		// bodies are regular PyCFGs — NOT NativeCFGs. The absence of any
		// NativeCFG member is the positive fingerprint that the call target
		// for `json.dumps(...)` in main.py is the user's parsed function, not
		// the library stub.
		Unit dumpsUnit = program.getUnit("json.dumps");
		assertNotNull("Local json.dumps must be registered on the Program", dumpsUnit);
		for (CodeMember m : ((FunctionUnit) dumpsUnit).getCodeMembers())
			assertFalse(
					"Local json.dumps must NOT be backed by a NativeCFG (found " + m + ")",
					m instanceof NativeCFG);

		Unit loadsUnit = program.getUnit("json.loads");
		assertNotNull("Local json.loads must be registered on the Program", loadsUnit);
		for (CodeMember m : ((FunctionUnit) loadsUnit).getCodeMembers())
			assertFalse(
					"Local json.loads must NOT be backed by a NativeCFG (found " + m + ")",
					m instanceof NativeCFG);
	}

	/**
	 * Exercises the ancestor-registration tweak in
	 * {@link it.unive.pylisa.frontend.PythonModuleImportManager#importModule}:
	 * importing {@code unspecced_pkg.submod.deep} should implicitly populate
	 * {@code PyModuleType} with every intermediate prefix as an unknown module,
	 * mirroring Python's {@code sys.modules} behavior for submodule imports.
	 */
	@Test
	public void importingUnresolvedDottedNameRegistersAncestorsAsUnknown() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/ancestors/main.py",
				false);
		translator.toLiSAProgram(true);

		// The full name is unresolved → unknown module, as before.
		assertTrue("unspecced_pkg.submod.deep must be registered",
				PyModuleType.isRegistered("unspecced_pkg.submod.deep"));
		assertTrue("unspecced_pkg.submod.deep must be marked unknown",
				PyModuleType.lookup("unspecced_pkg.submod.deep").isUnknown());

		// Ancestor prefixes must ALSO be registered as unknown modules — the
		// previous behavior left them un-registered, so a downstream type
		// query on `unspecced_pkg` or `unspecced_pkg.submod` would throw.
		assertTrue("ancestor unspecced_pkg must be registered",
				PyModuleType.isRegistered("unspecced_pkg"));
		assertTrue("ancestor unspecced_pkg must be marked unknown",
				PyModuleType.lookup("unspecced_pkg").isUnknown());

		assertTrue("ancestor unspecced_pkg.submod must be registered",
				PyModuleType.isRegistered("unspecced_pkg.submod"));
		assertTrue("ancestor unspecced_pkg.submod must be marked unknown",
				PyModuleType.lookup("unspecced_pkg.submod").isUnknown());
	}

	/**
	 * Exercises allocation-site abstraction for conditionally-redefined
	 * classes. Two {@code class Secret} blocks — one per branch of an
	 * {@code if/else} — must produce two distinct {@link PyClassUnit}s sharing
	 * the Python-visible base name but holding independent identities (the
	 * def-site suffix).
	 */
	@Test
	public void conditionalClassRedefinitionYieldsDistinctUnits() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/conditional_class/main.py",
				false);
		translator.toLiSAProgram(true);

		// Two distinct PyClassType entries must share the Python-visible base
		// name `__main__.Secret` — one per `class Secret:` statement in the
		// source.
		Collection<PyClassType> secrets = PyClassType.lookupAllByBaseName("__main__.Secret");
		assertEquals(
				"Expected two Secret def-sites (one per branch); got " + secrets,
				2,
				secrets.size());

		// Each def-site unit must carry the base name `__main__.Secret` while
		// its identity name includes the `@line:col` allocation-site suffix.
		for (PyClassType t : secrets) {
			Unit u = t.getUnit();
			assertTrue(
					"Expected PyClassUnit backing the Secret def-site; got " + u.getClass(),
					u instanceof PyClassUnit);
			PyClassUnit pcu = (PyClassUnit) u;
			assertEquals(
					"base name must be the Python-visible qualified name",
					"__main__.Secret",
					pcu.getBaseName());
			assertTrue(
					"identity name must include the `@line:col` allocation-site suffix; got " + pcu.getName(),
					pcu.getName().startsWith("__main__.Secret@"));
			assertNotEquals(
					"identity name must differ from base name",
					pcu.getBaseName(),
					pcu.getName());
		}

		// The two identity names must be distinct from each other.
		java.util.Set<String> identityNames = secrets.stream()
				.map(t -> t.getUnit().getName())
				.collect(java.util.stream.Collectors.toSet());
		assertEquals(
				"Expected two distinct identity names across the Secret def-sites",
				2,
				identityNames.size());
	}
}
