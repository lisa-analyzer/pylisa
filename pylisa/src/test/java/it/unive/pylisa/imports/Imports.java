package it.unive.pylisa.imports;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
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
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

public class Imports {
	// @Test DO NOT RUN THIS - TESTCASE IS A BROKEN PYTHON (can't do import
	// config.settings if settings is a class)
	public void imports2() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/import2/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("imports/import2");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");

		assertImports("imports/import2");
	}

	@Test
	public void imports3() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/import3/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("imports/import3");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");

		assertImports("imports/import3");
	}

	@Test
	public void imports4() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/import4/api.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("imports/import4");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");
		assertImports4("imports/import4");
	}

	@Test
	public void imports1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/import1/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("imports/import1");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");

		assertImports("imports/import1");
	}

	private void assertImports4(
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

		assertTrue(reportJson.isPresent(), "Missing __main__.$init() ");

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");
		assertNotNull(value);
		assertNotNull(type);
		assertNotNull(heap);
		assertNotNull(value.get("$__main__::z"), "$_main::z must be not null");
		assertEquals("\"3\"", value.get("$__main__::z").toString());
		assertEquals("\"3\"", value.get("$__main__::z").toString());
		System.out.println("[T] $__main__::y == [\"fastapi.APIRouter*\"]");
		assertEquals("[\"fastapi.APIRouter*\"]", type.get("$__main__::y").toString());
	}

	private void assertImports(
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

		assertTrue(reportJson.isPresent(), "Missing __main__.$init() ");

		ObjectMapper mapper = new ObjectMapper();

		JsonNode root = mapper.readTree(reportJson.get().toFile());
		int nodesCount = root.get("descriptions").size();
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("normal")
				.get("state").get("Analysis State");
		JsonNode heap = exitAnalysisState.get("heap");
		JsonNode type = exitAnalysisState.get("type");
		JsonNode value = exitAnalysisState.get("value");
		assertNotNull(value);
		assertNotNull(type);
		assertNotNull(heap);
		assertNotNull(value.get("$__main__::x"), "$_main::x must be not null");
		// Class-member keys now carry the def-site suffix (e.g.
		// `$config.Settings@10:0::DEBUG`) because each `class` statement mints
		// a fresh allocation-site identity. Match the base prefix rather than
		// the exact name so the assertion stays stable across source edits
		// that shift line numbers.
		JsonNode settingsDebug = findFirstFieldMatching(value, "$config.Settings@", "::DEBUG");
		assertNotNull(settingsDebug, "$config.Settings@…::DEBUG must be not null");
		assertNotNull(value.get("$__main__::z"), "$_main::z must be not null");
		assertEquals("\"3\"", value.get("$__main__::x").toString(), "$__main__::x is not 3.");
		assertEquals("\"true\"", settingsDebug.toString(), "$config.Settings@…::DEBUG is not true.");
		assertEquals("\"true\"", value.get("$__main__::z").toString(), "$__main__::z is not true.");
		assertNull(type.get("$config.settings"),
				"$config.settings should not be present, only $config::settings (probabaly, a problem with import.");
	}

	private static JsonNode findFirstFieldMatching(
			JsonNode container,
			String prefix,
			String suffix) {
		if (container == null || !container.isObject())
			return null;
		java.util.Iterator<String> names = container.fieldNames();
		while (names.hasNext()) {
			String name = names.next();
			if (name.startsWith(prefix) && name.endsWith(suffix))
				return container.get(name);
		}
		return null;
	}
}
