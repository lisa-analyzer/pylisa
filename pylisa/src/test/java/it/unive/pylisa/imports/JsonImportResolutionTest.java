package it.unive.pylisa.imports;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
import org.junit.jupiter.api.Test;

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
				LibrarySpecificationProvider.isLibraryLoaded("json"),
				"Expected the stdlib `json` spec to be loaded when no local json.py exists");

		ModuleUnit jsonUnit = (ModuleUnit) PyModuleType.lookup("json").getUnit();
		assertNotNull(jsonUnit, "json ModuleUnit must be registered");

		// Library-spec units carry a SourceCodeLocation built from `location
		// json`.
		CodeLocation loc = jsonUnit.getLocation();
		assertTrue(
				loc instanceof SourceCodeLocation,
				"Expected json unit to come from the library spec (SourceCodeLocation); got " + loc);
		assertEquals(
				"json",
				((SourceCodeLocation) loc).getSourceFile(),
				"Library-spec unit should carry the `location json` declared in json.txt");
		assertFalse(
				loc == SyntheticLocation.INSTANCE,
				"Stub-backed json unit must not carry SyntheticLocation");

		// The two stub methods were registered on the Program under their
		// qualified names during library loading.
		Unit dumpsUnit = program.getUnit("json.dumps");
		assertNotNull(dumpsUnit, "Library-backed json.dumps FunctionUnit must be registered");
		assertNotNull(
				program.getUnit("json.loads"),
				"Library-backed json.loads FunctionUnit must be registered");

		// Stub bodies are NativeCFGs wrapping the NoOpFunction implementation
		// declared in json.txt. This is the positive fingerprint that
		// `json.dumps(...)` in main.py is dispatched to the stub, not to a
		// parsed function.
		assertTrue(
				((FunctionUnit) dumpsUnit).getCodeMembers().stream().anyMatch(m -> m instanceof NativeCFG),
				"json.dumps body must be a NativeCFG (stub wrapping NoOpFunction); code members: "
						+ ((FunctionUnit) dumpsUnit).getCodeMembers());
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
				LibrarySpecificationProvider.isLibraryLoaded("json"),
				"Local json.py must shadow the stdlib `json` library spec");

		ModuleUnit jsonUnit = (ModuleUnit) PyModuleType.lookup("json").getUnit();
		assertNotNull(jsonUnit, "json ModuleUnit must be registered");

		// loadProjectModule stamps SyntheticLocation on project-file-backed
		// units — the distinguishing fingerprint vs. the library-spec unit
		// (which would carry a SourceCodeLocation("json", 0, 0)).
		CodeLocation loc = jsonUnit.getLocation();
		assertTrue(
				loc == SyntheticLocation.INSTANCE,
				"Expected json unit to come from loadProjectModule (SyntheticLocation); got " + loc);

		// `def dumps` and `def loads` from json.py become FunctionUnits whose
		// bodies are regular PyCFGs — NOT NativeCFGs. The absence of any
		// NativeCFG member is the positive fingerprint that the call target
		// for `json.dumps(...)` in main.py is the user's parsed function, not
		// the library stub.
		Unit dumpsUnit = program.getUnit("json.dumps");
		assertNotNull(dumpsUnit, "Local json.dumps must be registered on the Program");
		for (CodeMember m : ((FunctionUnit) dumpsUnit).getCodeMembers())
			assertFalse(
					m instanceof NativeCFG,
					"Local json.dumps must NOT be backed by a NativeCFG (found " + m + ")");

		Unit loadsUnit = program.getUnit("json.loads");
		assertNotNull(loadsUnit, "Local json.loads must be registered on the Program");
		for (CodeMember m : ((FunctionUnit) loadsUnit).getCodeMembers())
			assertFalse(
					m instanceof NativeCFG,
					"Local json.loads must NOT be backed by a NativeCFG (found " + m + ")");
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
		assertTrue(PyModuleType.isRegistered("unspecced_pkg.submod.deep"),
				"unspecced_pkg.submod.deep must be registered");
		assertTrue(PyModuleType.lookup("unspecced_pkg.submod.deep").isUnknown(),
				"unspecced_pkg.submod.deep must be marked unknown");

		// Ancestor prefixes must ALSO be registered as unknown modules — the
		// previous behavior left them un-registered, so a downstream type
		// query on `unspecced_pkg` or `unspecced_pkg.submod` would throw.
		assertTrue(PyModuleType.isRegistered("unspecced_pkg"),
				"ancestor unspecced_pkg must be registered");
		assertTrue(PyModuleType.lookup("unspecced_pkg").isUnknown(),
				"ancestor unspecced_pkg must be marked unknown");

		assertTrue(PyModuleType.isRegistered("unspecced_pkg.submod"),
				"ancestor unspecced_pkg.submod must be registered");
		assertTrue(PyModuleType.lookup("unspecced_pkg.submod").isUnknown(),
				"ancestor unspecced_pkg.submod must be marked unknown");
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
				2,
				secrets.size(),
				"Expected two Secret def-sites (one per branch); got " + secrets);

		// Each def-site unit must carry the base name `__main__.Secret` while
		// its identity name includes the `@line:col` allocation-site suffix.
		for (PyClassType t : secrets) {
			Unit u = t.getUnit();
			assertTrue(
					u instanceof PyClassUnit,
					"Expected PyClassUnit backing the Secret def-site; got " + u.getClass());
			PyClassUnit pcu = (PyClassUnit) u;
			assertEquals(
					"__main__.Secret",
					pcu.getBaseName(),
					"base name must be the Python-visible qualified name");
			assertTrue(
					pcu.getName().startsWith("__main__.Secret@"),
					"identity name must include the `@line:col` allocation-site suffix; got " + pcu.getName());
			assertNotEquals(
					pcu.getBaseName(),
					pcu.getName(),
					"identity name must differ from base name");
		}

		// The two identity names must be distinct from each other.
		java.util.Set<String> identityNames = secrets.stream()
				.map(t -> t.getUnit().getName())
				.collect(java.util.stream.Collectors.toSet());
		assertEquals(
				2,
				identityNames.size(),
				"Expected two distinct identity names across the Secret def-sites");
	}
}
