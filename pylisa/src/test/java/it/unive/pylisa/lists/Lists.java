package it.unive.pylisa.lists;

import static it.unive.pylisa.testutil.LiSAConfigs.getDefaultConf;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Specifications of lists modelled element by element: the tests run the full
 * pipeline on tiny programs and check the concrete values of list literals,
 * {@code __getitem__} and {@code __len__} at the program's exit. pylisa does
 * not model lists element by element yet: reading an element or the length of
 * a list gives an unknown value (sound, imprecise). Tagged as known to fail;
 * the build fails when one of them starts to pass, so that the tag is
 * removed.
 */
@Tag("known-failing")
public class Lists {

	@Test
	public void literalIntList() throws IOException {
		// x = [3, 4, 5]; y = x[2]; z = len(x)
		runAnd("py-testcases/lists/literal.py", "lists/literal", root -> {
			JsonNode value = exitValue(root);
			assertConcrete(value, "y", "5");
			assertConcrete(value, "z", "3");

			// the underlying allocation site should hold each element strongly
			JsonNode type = exitType(root);
			assertTrue(siteHasField(type, "[0]"), "expected cell [0] for x[0]");
			assertTrue(siteHasField(type, "[1]"), "expected cell [1] for x[1]");
			assertTrue(siteHasField(type, "[2]"), "expected cell [2] for x[2]");
			assertTrue(siteHasField(type, "[->size]"),
					"expected cell [->size] holding the length");
		});
	}

	@Test
	public void literalStringList() throws IOException {
		// methods = ["GET", "POST"]; m0 = methods[0]; m1 = methods[1]; n = len(methods)
		runAnd("py-testcases/lists/strings.py", "lists/strings", root -> {
			JsonNode value = exitValue(root);
			assertConcrete(value, "m0", "\"GET\"");
			assertConcrete(value, "m1", "\"POST\"");
			assertConcrete(value, "n", "2");
		});
	}

	@Test
	public void emptyList() throws IOException {
		// l = []; n = len(l)
		runAnd("py-testcases/lists/empty.py", "lists/empty", root -> {
			JsonNode value = exitValue(root);
			assertConcrete(value, "n", "0");
		});
	}

	/**
	 * Stress test for the internal/user namespace split:
	 * <pre>
	 * x = list()
	 * x.append(5)
	 * x.size = 100
	 * f = len(x)   # 1 — from the internal $size cell, bumped by append
	 * g = x.size   # 100 — from the user-attribute "size" cell
	 * </pre>
	 * The two cells must not collide. Internal {@code $size} is keyed via the
	 * {@link it.unive.pylisa.symbolic.InternalAttribute} wrapper and renders as
	 * {@code [->size]}; the user attribute is keyed via a {@code Variable("size")}
	 * child and renders as {@code [size]} — distinct identifiers in the heap.
	 */
	@Test
	public void appendAndUserAttributeAreDisjoint() throws IOException {
		runAnd("py-testcases/lists/append_attr.py", "lists/append_attr", root -> {
			JsonNode value = exitValue(root);
			assertConcrete(value, "f", "1");
			assertConcrete(value, "g", "100");

			// internal size cell carries the post-append value
			assertSiteValue(value, "[->size]", "1");
			// user attribute cell carries the user-assigned value, in its own slot
			assertSiteValue(value, "[size]", "100");
		});
	}

	// --- helpers -----------------------------------------------------------

	@FunctionalInterface
	private interface ReportAssertion {
		void check(
				JsonNode root)
				throws IOException;
	}

	private void runAnd(
			String testcase,
			String workdir,
			ReportAssertion assertion)
			throws IOException {
		PyFrontend translator = new PyFrontend(testcase, false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getDefaultConf(workdir);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing builtins.object.__new__");

		Path outputDir = Path.of("tests", workdir);
		Optional<Path> reportJson;
		try (Stream<Path> files = Files.list(outputDir)) {
			reportJson = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith(".graph.json"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}
		assertTrue(reportJson.isPresent(), "missing __main__.$init() graph json under tests/" + workdir);

		ObjectMapper mapper = new ObjectMapper();
		JsonNode root = mapper.readTree(reportJson.get().toFile());
		assertion.check(root);
	}

	private JsonNode exitState(
			JsonNode root) {
		JsonNode descriptions = root.get("descriptions");
		int n = descriptions.size();
		return descriptions.get(n - 1).get("description").get("state");
	}

	private JsonNode exitValue(
			JsonNode root) {
		JsonNode v = exitState(root).get("value");
		assertNotNull(v, "missing value section in exit state");
		assertTrue(v.isObject(),
				"value section is not an object — likely TOP/BOTTOM, see " + v.toString());
		return v;
	}

	private JsonNode exitType(
			JsonNode root) {
		JsonNode t = exitState(root).get("type");
		assertNotNull(t, "missing type section in exit state");
		return t;
	}

	private void assertConcrete(
			JsonNode value,
			String varName,
			String expected) {
		String key = "$__main__::" + varName;
		JsonNode v = value.get(key);
		assertNotNull(v, "expected variable " + key + " in the value section, got keys: "
				+ keysOf(value));
		assertEquals(expected, v.asText(), "wrong value for " + key);
	}

	private void assertSiteValue(
			JsonNode section,
			String fieldSuffix,
			String expected) {
		Iterator<Map.Entry<String, JsonNode>> it = section.fields();
		while (it.hasNext()) {
			Map.Entry<String, JsonNode> e = it.next();
			if (e.getKey().endsWith(fieldSuffix)) {
				assertEquals(expected, e.getValue().asText(),
						"wrong value at heap cell ending in " + fieldSuffix);
				return;
			}
		}
		throw new AssertionError("no heap cell ending in " + fieldSuffix
				+ " — keys: " + keysOf(section));
	}

	private boolean siteHasField(
			JsonNode section,
			String fieldSuffix) {
		if (section == null || !section.isObject())
			return false;
		Iterator<Map.Entry<String, JsonNode>> it = section.fields();
		while (it.hasNext())
			if (it.next().getKey().endsWith(fieldSuffix))
				return true;
		return false;
	}

	private String keysOf(
			JsonNode object) {
		if (object == null || !object.isObject())
			return "<not an object>";
		StringBuilder sb = new StringBuilder("[");
		Iterator<Map.Entry<String, JsonNode>> it = object.fields();
		while (it.hasNext()) {
			sb.append(it.next().getKey());
			if (it.hasNext())
				sb.append(", ");
		}
		return sb.append("]").toString();
	}
}
