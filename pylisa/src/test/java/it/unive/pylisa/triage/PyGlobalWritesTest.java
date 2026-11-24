package it.unive.pylisa.triage;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.libraries.dynamic.Eval;
import it.unive.pylisa.libraries.dynamic.Exec;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * Tests for {@link PyGlobalWrites}.
 */
public class PyGlobalWritesTest {

	private static final String SOURCE = String.join("\n",
			"import math",
			"count = 0",
			"class C:",
			"    pass",
			"def inc():",
			"    global count",
			"    count += 1",
			"def reader():",
			"    return count + 1",
			"def local_only():",
			"    count = 5",
			"    return count",
			"def deleter():",
			"    global count",
			"    del count",
			"def setter():",
			"    C.attr = 3",
			"def tuple_setter():",
			"    C.other, y = 1, 2",
			"    return y",
			"def importer():",
			"    global math2",
			"    import math as math2",
			"def loop():",
			"    global i",
			"    for i in range(3):",
			"        pass",
			"inc()",
			"");

	@Rule
	public TemporaryFolder folder = new TemporaryFolder();

	private Program parse() throws Exception {
		return parse("globals.py", SOURCE);
	}

	private Program parse(
			String name,
			String source)
			throws Exception {
		File file = folder.newFile(name);
		Files.writeString(file.toPath(), source, StandardCharsets.UTF_8);
		return new PyFrontend().parseFromListOfFile(List.of(file.toString()));
	}

	private static CFG cfg(
			Program program,
			String name) {
		for (CFG cfg : program.getAllCFGs())
			if (cfg.getDescriptor().getName().equals(name))
				return cfg;
		throw new AssertionError("no cfg named " + name);
	}

	/**
	 * The names written by the code of the given function, without the module
	 * qualifying them.
	 */
	private static Set<String> written(
			PyGlobalWrites writes,
			Program program,
			String name) {
		Set<String> written = writes.writtenBy(cfg(program, name));
		assertNotNull("the writes of " + name + " should be known", written);
		return written.stream().map(n -> n.substring(n.lastIndexOf('.') + 1)).collect(Collectors.toSet());
	}

	@Test
	public void globalWritesOfFunctions() throws Exception {
		Program program = parse();
		PyGlobalWrites writes = new PyGlobalWrites();
		assertEquals(Set.of("count"), written(writes, program, "inc"));
		assertEquals(Set.of(), written(writes, program, "reader"));
		assertEquals(Set.of(), written(writes, program, "local_only"));
		assertEquals(Set.of("count"), written(writes, program, "deleter"));
		assertEquals(Set.of("attr"), written(writes, program, "setter"));
		// also inside a tuple of targets
		assertEquals(Set.of("other"), written(writes, program, "tuple_setter"));
		assertEquals(Set.of("math2"), written(writes, program, "importer"));
		assertEquals(Set.of("i"), written(writes, program, "loop"));
		// the module binds its names
		Set<String> module = writes.writtenBy(cfg(program, "$main")).stream()
				.map(n -> n.substring(n.lastIndexOf('.') + 1)).collect(Collectors.toSet());
		assertTrue(module.toString(), module.containsAll(Set.of("math", "count", "C", "inc", "reader")));
		// the names are the ones of the global variables of the module
		assertTrue(writes.writtenBy(cfg(program, "inc")).iterator().next().endsWith("globals.count"));
	}

	@Test
	public void globalsOfTheProgram() throws Exception {
		Program program = parse();
		Set<String> globals = new PyGlobalWrites().globalsOf(program);
		Set<String> simple = globals.stream().map(n -> n.substring(n.lastIndexOf('.') + 1))
				.collect(Collectors.toSet());
		// the names of the module, also the ones that are only read, and the
		// attributes assigned through classes
		assertTrue(simple.toString(), simple.containsAll(Set.of("count", "math", "math2", "i", "C", "inc", "attr")));
		assertTrue(globals.toString(), globals.stream().anyMatch(n -> n.endsWith("globals.count")));
		// the local variables of the functions are not global
		assertEquals(1, globals.stream().filter(n -> n.endsWith(".count")).count());
	}

	@Test
	public void starImportsWriteUnknownGlobals() throws Exception {
		// the names bound by star imports are not all known
		Program program = parse("starred.py", "from math import *\nx = 1\n");
		assertNull(new PyGlobalWrites().writtenBy(cfg(program, "$main")));
	}

	// exec and eval might bind or delete any name of the module
	private static boolean havocking(
			NativeCFG member) {
		return member.getConstruct() == Exec.class || member.getConstruct() == Eval.class;
	}

	@Test
	public void nativesWriteNoGlobalExceptExec() throws Exception {
		Program program = parse();
		PyGlobalWrites writes = new PyGlobalWrites();
		boolean exec = false;
		for (CodeMember member : program.getCodeMembers())
			if (member instanceof NativeCFG) {
				if (havocking((NativeCFG) member)) {
					exec = true;
					assertNull(writes.writtenBy(member));
				} else
					assertEquals(Set.of(), writes.writtenBy(member));
			}
		for (it.unive.lisa.program.Unit unit : program.getUnits())
			for (CodeMember member : unit.getCodeMembers())
				if (member instanceof NativeCFG && havocking((NativeCFG) member)) {
					exec = true;
					assertNull(writes.writtenBy(member));
				}
		assertTrue("exec should be a native of the program", exec);
	}
}
