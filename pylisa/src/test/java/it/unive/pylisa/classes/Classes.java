package it.unive.pylisa.classes;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;
import static org.junit.Assert.*;
import static org.junit.Assert.assertNotEquals;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.program.FunctionUnit;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.Test;

public class Classes {

	@Test
	public void class1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/classes/class1.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("classes/class1");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

		assertClass1Correctness("classes/class1");
	}

	@Test
	public void classGlobalFields() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/classes/global_fields.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("classes/global_fields");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

		assertClassGlobalsCorrectness("classes/global_fields");
	}

	private void assertClassGlobalsCorrectness(
			String workdir)
			throws IOException {
		Path outputDir = Path.of("tests", workdir);
		Optional<Path> reportJson;
		try (Stream<Path> files = Files.list(outputDir)) {
			reportJson = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith(".graph.json"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}

		assertTrue("Missing __main__.$init() ht", reportJson.isPresent());

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");

		System.out.println("[H] $__main__::config == [\"heap[s]:pp@'py-testcases/classes/global_fields.py':14:16\"]");
		assertEquals("[\"heap[s]:pp@'py-testcases/classes/global_fields.py':14:16\"]",
				heap.get("$__main__::config").toString());

		System.out.println("[T] $__main__.Config::case_sensitive == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__.Config::case_sensitive").toString());

		System.out.println("[T] $__main__.Config::env_file == [\"string\"]");
		assertEquals("[\"string\"]", type.get("$__main__.Config::env_file").toString());

		System.out.println("[T] $__main__.Config::env_file_encoding == [\"bool\"]");
		assertEquals("[\"string\"]", type.get("$__main__.Config::env_file_encoding").toString());

		System.out.println("[T] $__main__::a == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__::a").toString());

		System.out.println("[T] $__main__::b == [\"bool\"]");
		assertEquals("[\"string\"]", type.get("$__main__::b").toString());

		System.out.println("[T] $__main__::c == [\"string\"]");
		assertEquals("[\"string\"]", type.get("$__main__::c").toString());

		System.out.println("[T] $__main__::d == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__::d").toString());

		System.out.println("[T] $__main__::e == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__::e").toString());

		System.out.println("[T] $__main__::f == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__::f").toString());

		System.out.println("[T] $__main__::w == [\"bool\"]");
		assertEquals("[\"bool\"]", type.get("$__main__::w").toString());

		System.out.println("[T] $__main__::x == [\"bool\"]");
		assertEquals("[\"string\"]", type.get("$__main__::x").toString());

		System.out.println("[T] $__main__::y == [\"bool\"]");
		assertEquals("[\"string\"]", type.get("$__main__::y").toString());

		System.out.println("[T] $__main__::z == [\"bool\"]");
		assertEquals("[\"string\"]", type.get("$__main__::z").toString());

		System.out.println("[V] $__main__::a == \"false\"");
		assertEquals("\"false\"", value.get("$__main__::a").toString());

		System.out.println("[V] $__main__::b == \".env\"");
		assertEquals("\"\\\".env\\\"\"", value.get("$__main__::b").toString());

		System.out.println("[V] $__main__::c == \"utf-8\"");
		assertEquals("\"\\\"utf-8\\\"\"", value.get("$__main__::c").toString());

		System.out.println("[V] $__main__::d == \"false\"");
		assertEquals("\"false\"", value.get("$__main__::d").toString());

		System.out.println("[V] $__main__::e == \"true\"");
		assertEquals("\"true\"", value.get("$__main__::e").toString());

		System.out.println("[V] $__main__::f == \"false\"");
		assertEquals("\"false\"", value.get("$__main__::f").toString());

		System.out.println("[V] $__main__::w == \"true\"");
		assertEquals("\"true\"", value.get("$__main__::w").toString());

		System.out.println("[V] $__main__::x == \".env\"");
		assertEquals("\"\\\".env\\\"\"", value.get("$__main__::x").toString());

		System.out.println("[V] $__main__::y == \"utf-8\"");
		assertEquals("\"\\\"utf-8\\\"\"", value.get("$__main__::y").toString());

		System.out.println("[V] $__main__::z == \"utf-8\"");
		assertEquals("\"\\\"utf-8\\\"\"", value.get("$__main__::z").toString());

		System.out
				.println("[V] heap[s]:pp@'py-testcases/classes/global_fields.py':14:16[case_sensitive] == [\"true\"]");
		assertEquals("\"true\"",
				value.get("heap[s]:pp@'py-testcases/classes/global_fields.py':14:16[case_sensitive]").toString());

		System.out.println("[V] $__main__.Config::case_sensitive == \"false\"");
		assertEquals("\"false\"", value.get("$__main__.Config::case_sensitive").toString());

		System.out.println("[V] $__main__.Config::env_file == \".env\"");
		assertEquals("\"\\\".env\\\"\"", value.get("$__main__.Config::env_file").toString());

		System.out.println("[V] $__main__.Config::env_file_encoding == \"utf-8\"");
		assertEquals("\"\\\"utf-8\\\"\"", value.get("$__main__.Config::env_file_encoding").toString());
	}

	private static void assertClass1Correctness(
			String workdir)
			throws IOException {
		Path outputDir = Path.of("tests", workdir);
		Optional<Path> reportJson;
		try (Stream<Path> files = Files.list(outputDir)) {
			reportJson = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith(".graph.json"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}

		assertTrue("Missing __main__.$init() ht", reportJson.isPresent());

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");

		System.out.println("[H] Assert $__main__::class1 == [\"heap[s]:pp@'py-testcases/classes/class1.py':7:16\"]");
		assertEquals("[\"heap[s]:pp@'py-testcases/classes/class1.py':7:16\"]",
				heap.get("$__main__::class1").toString());

		System.out.println("[T] Assert $__main__.Class1 == [\"__main__.Class1*\"]");
		assertEquals("[\"__main__.Class1\"]", type.get("$__main__.Class1").toString());

		System.out.println("[T] Assert $__main__::class1 == [\"__main__.Class1*\"]");
		assertEquals("[\"__main__.Class1*\"]", type.get("$__main__::class1").toString());

		System.out.println("[T] Assert $__main__.Class1::test == [\"__main__.Class1.test\"]");

		assertEquals("[\"__main__.Class1*\"]", type.get("$__main__::class1").toString());
		System.out.println("[T] heap[s]:pp@'py-testcases/classes/class1.py':7:16[x] == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[x]").toString());

		System.out.println("[T] $__main__::f == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::f").toString());

		System.out.println("[T] Assert $__main__::a == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::a").toString());

		System.out.println("[T] Assert $__main__::b == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::b").toString());

		System.out.println("[T] Assert $__main__::c == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::c").toString());

		System.out.println("[T] Assert $__main__::d == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::d").toString());

		System.out.println("[T] Assert $__main__::e == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::e").toString());

		System.out.println("[T] Assert $__main__::g == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("$__main__::g").toString());

		System.out.println("[T] $__main__::h == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[x]").toString());

		System.out.println("[T] $__main__::i == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[x]").toString());

		System.out.println("[T] Assert heap[s]:pp@'py-testcases/classes/class1.py':7:16[Y] == [\"int32\"]");
		assertEquals("[\"int32\"]", type.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[Y]").toString());

		System.out.println("[V] Assert $__main__::f == \"10\"");
		assertEquals("\"10\"", value.get("$__main__::f").toString());

		System.out.println("[V] Assert heap[s]:pp@'py-testcases/classes/class1.py':7:16[x] == \"10\"");
		assertEquals("\"10\"", value.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[x]").toString());

		System.out.println("[V] Assert $__main__.Class1::Y == \"100\"");
		assertEquals("\"100\"", value.get("$__main__.Class1::Y").toString());

		System.out.println("[V] Assert $__main__::a == \"20\"");
		assertEquals("\"20\"", value.get("$__main__::a").toString());

		System.out.println("[V] Assert $__main__::b == \"20\"");
		assertEquals("\"20\"", value.get("$__main__::b").toString());

		System.out.println("[V] Assert $__main__::c == \"100\"");
		assertEquals("\"100\"", value.get("$__main__::c").toString());

		System.out.println("[V] Assert $__main__::d == \"100\"");
		assertEquals("\"100\"", value.get("$__main__::d").toString());

		System.out.println("[V] Assert $__main__::e == \"200\"");
		assertEquals("\"200\"", value.get("$__main__::e").toString());

		System.out.println("[V] Assert $__main__::g == \"100\"");
		assertEquals("\"100\"", value.get("$__main__::g").toString());

		System.out.println("[V] Assert heap[s]:pp@'py-testcases/classes/class1.py':7:16[Y] == \"200\"");
		assertEquals("\"200\"", value.get("heap[s]:pp@'py-testcases/classes/class1.py':7:16[Y]").toString());

		System.out.println("Assert [V] $__main__::h == \"70\"");
		assertEquals("\"70\"", value.get("$__main__::h").toString());

		System.out.println("Assert [V] $__main__::i == \"70\"");
		assertEquals("\"70\"", value.get("$__main__::i").toString());
	}

	private static void assertClassMethodCorrect(
			String workdir)
			throws IOException {
		Path outputDir = Path.of("tests", workdir);
		Optional<Path> reportJson;
		try (Stream<Path> files = Files.list(outputDir)) {
			reportJson = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith(".graph.json"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}

		assertTrue("Missing __main__.$init() missing", reportJson.isPresent());

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");

		System.out.println("[H] Assert $__main__::a == [\"heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6\"]");
		assertEquals("[\"heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6\"]",
				heap.get("$__main__::a").toString());

		assertNull(type.get("$rec@'py-testcases/classes/staticmethod.py':9:10"));
		assertNull(type.get("$rec@'py-testcases/classes/staticmethod.py':10:10"));

		assertNotEquals("\"#TOP#\"", type.get("$__main__.A::foo"));

		System.out.println("[T] Assert $builtins::staticmethod == [\"builtins.staticmethod\"]");
		assertEquals("[\"builtins.staticmethod\"]", type.get("$builtins::staticmethod").toString());

		System.out.println("[T] Assert heap[s]:pp@'py-testcases/classes/staticmethod.py':4:4 is null");
		assertNull(type.get("heap[s]:pp@'py-testcases/classes/staticmethod.py':4:4"));

		System.out.println("Assert [V] heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6[x] == \"10\"");
		assertEquals("\"10\"", value.get("heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6[x]").toString());

		System.out.println("Assert [V] heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6[x] == \"10\"");
		assertEquals("\"10\"", value.get("heap[s]:pp@'py-testcases/classes/staticmethod.py':8:6[x]").toString());

		System.out.println("Assert [V] $__main__::b == \"20\"");
		assertEquals("\"20\"", value.get("$__main__::b").toString());

		System.out.println("Assert [V] $__main__::c == \"20\"");
		assertEquals("\"20\"", value.get("$__main__::c").toString());

		System.out.println("Assert [V] $__main__::d == \"10\"");
		assertEquals("\"10\"", value.get("$__main__::d").toString());

		System.out.println("Assert [V] $__main__::e == \"10\"");
		assertEquals("\"10\"", value.get("$__main__::e").toString());
	}

	// @Test
	public void class2() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/classes/class2.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("classes/class2");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void staticMethod() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/classes/staticmethod.py",
				false);
		Program program = translator.toLiSAProgram(false);
		assertNotNull("missing functionunit builtins.staticmethod.", program.getUnit("builtins.staticmethod"));
		assertTrue("builtins.staticmethod should be of type FunctionUnit.",
				program.getUnit("builtins.staticmethod") instanceof FunctionUnit);
		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));
		LiSAConfiguration conf = getLisaConf("classes/staticmethod");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
		assertClassMethodCorrect("classes/staticmethod");
	}

	@Test
	public void superClasses() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/classes/classes_super.py",
				false);
		Program program = translator.toLiSAProgram(false);
		assertNotNull("missing functionunit builtins.staticmethod.", program.getUnit("builtins.staticmethod"));
		assertTrue("builtins.staticmethod should be of type FunctionUnit.",
				program.getUnit("builtins.staticmethod") instanceof FunctionUnit);
		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		// assertNotNull("missing super builtins.object.super.",
		// program.getUnit("builtins.object.super"));
		LiSAConfiguration conf = getLisaConf("classes/classes_super");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
		assertSuperClassesCorrect("classes/classes_super");
	}

	private static void assertSuperClassesCorrect(
			String workdir)
			throws IOException {
		Path outputDir = Path.of("tests", workdir);
		Optional<Path> reportJson;
		try (Stream<Path> files = Files.list(outputDir)) {
			reportJson = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith(".graph.json"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}

		assertTrue("Missing __main__.$init() missing", reportJson.isPresent());

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");

		System.out.println("Assert [V] $__main__::a == \"1000\"");
		assertEquals("\"1000\"", value.get("$__main__::a").toString());

		System.out.println("Assert [V] $__main__::b == \"2000\"");
		assertEquals("\"2000\"", value.get("$__main__::b").toString());
	}

	/**
	 * Verifies LEGB scope isolation for class definitions:
	 * <ul>
	 * <li>the program parses without exception</li>
	 * <li>{@code Foo.method_local} is compiled as a distinct {@link Unit}</li>
	 * <li>the {@code Foo} class unit exists and carries at least one ancestor
	 * ({@code builtins.object}), confirming that the implicit-object-ancestor
	 * injection path in {@code visitClassdef} ran correctly</li>
	 * </ul>
	 */
	// @Test
	public void testLEGBScopeIsolation() throws IOException {
		String path = "py-testcases/classes/class_scope_legb.py";
		PyFrontend frontend = new PyFrontend(path, false);
		Program program = frontend.toLiSAProgram(false);

		// Verify the program parses without exception and has the expected
		// units
		assertNotNull(program);

		// Find the FunctionUnit for Foo.method_local
		Unit methodUnit = program.getUnits().stream()
				.filter(u -> u.getName().contains("method_local"))
				.findFirst()
				.orElse(null);
		assertNotNull("method_local unit must exist", methodUnit);

		// Find the ClassUnit for Foo and verify it has builtins.object as
		// ancestor
		it.unive.lisa.program.CompilationUnit fooUnit = (it.unive.lisa.program.CompilationUnit) program.getUnits()
				.stream()
				.filter(u -> u.getName().endsWith(".Foo"))
				.findFirst()
				.orElse(null);
		assertNotNull("Foo class unit must exist", fooUnit);
		assertFalse("Foo must have at least one ancestor (builtins.object)",
				fooUnit.getImmediateAncestors().isEmpty());
	}

}
