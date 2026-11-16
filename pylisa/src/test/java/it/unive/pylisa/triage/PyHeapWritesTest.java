package it.unive.pylisa.triage;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.triage.model.HeapWrites.Written;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.libraries.dynamic.Exec;
import it.unive.pylisa.libraries.lists.ListAppend;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * Tests for {@link PyHeapWrites}.
 */
public class PyHeapWritesTest {

	private static final String SOURCE = String.join("\n",
			"class P:",
			"    def __init__(self):",
			"        self.count = 0",
			"        self.label = 'x'",
			"def bump(p):",
			"    p.count += 1",
			"def rename(p):",
			"    p.label = 'y'",
			"def neither(p, xs):",
			"    xs.append(1)",
			"    return len(xs)",
			"p = P()",
			"bump(p)",
			"rename(p)",
			"neither(p, [])",
			"");

	@Rule
	public TemporaryFolder folder = new TemporaryFolder();

	private Program parse() throws Exception {
		File file = folder.newFile("heap.py");
		Files.writeString(file.toPath(), SOURCE, StandardCharsets.UTF_8);
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

	private static NativeCFG nativeOf(
			Program program,
			Class<?> construct) {
		for (CodeMember member : program.getCodeMembers())
			if (member instanceof NativeCFG && ((NativeCFG) member).getConstruct() == construct)
				return (NativeCFG) member;
		for (Unit unit : program.getUnits()) {
			for (CodeMember member : unit.getCodeMembers())
				if (member instanceof NativeCFG && ((NativeCFG) member).getConstruct() == construct)
					return (NativeCFG) member;
			if (unit instanceof it.unive.lisa.program.CompilationUnit)
				for (CodeMember member : ((it.unive.lisa.program.CompilationUnit) unit)
						.getInstanceCodeMembers(true))
					if (member instanceof NativeCFG && ((NativeCFG) member).getConstruct() == construct)
						return (NativeCFG) member;
		}
		throw new AssertionError("no native " + construct.getSimpleName());
	}

	@Test
	public void attributesWrittenByFunctions() throws Exception {
		Program program = parse();
		PyHeapWrites writes = new PyHeapWrites();
		Written bump = writes.writtenBy(cfg(program, "bump"));
		assertNotNull(bump);
		assertTrue(bump.writesField("count"));
		assertFalse(bump.writesField("label"));
		// also qualified by a class
		assertFalse(bump.writesField("P::label"));
		// internal fields and cells might be written by any code
		assertTrue(bump.writesField("$length"));
		assertTrue(bump.writesCells());

		Written rename = writes.writtenBy(cfg(program, "rename"));
		assertTrue(rename.writesField("label"));
		assertFalse(rename.writesField("count"));

		// the natives of lists write no attribute of the program
		Written neither = writes.writtenBy(cfg(program, "neither"));
		assertFalse(neither.writesField("count"));
		assertFalse(neither.writesField("label"));
		assertTrue(neither.writesField("$length"));
		Written append = writes.writtenBy(nativeOf(program, ListAppend.class));
		assertNotNull(append);
		assertFalse(append.writesField("count"));
		assertTrue(append.writesCells());
	}

	@Test
	public void unknownCodeWritesAnything() throws Exception {
		Program program = parse();
		assertNull(new PyHeapWrites().writtenBy(nativeOf(program, Exec.class)));
	}

	@Test
	public void summariesAndArrayProperties() {
		PyHeapWrites writes = new PyHeapWrites();
		assertTrue(writes.summaryChildren().contains(ListCreation.UNKNOWN_INDEX));
		assertTrue(writes.arrayProperties().isEmpty());
	}
}
