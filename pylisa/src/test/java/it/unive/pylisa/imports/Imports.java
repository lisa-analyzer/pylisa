package it.unive.pylisa.imports;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;
import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

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
import org.junit.Test;

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

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

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

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

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

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));
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

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

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

		assertTrue("Missing __main__.$init() ", reportJson.isPresent());

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
		assertNotNull("$_main::z must be not null", value.get("$__main__::z"));
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

		assertTrue("Missing __main__.$init() ", reportJson.isPresent());

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
		assertNotNull("$_main::x must be not null", value.get("$__main__::x"));
		// Class-member keys now carry the def-site suffix (e.g.
		// `$config.Settings@10:0::DEBUG`) because each `class` statement mints
		// a fresh allocation-site identity. Match the base prefix rather than
		// the exact name so the assertion stays stable across source edits
		// that shift line numbers.
		JsonNode settingsDebug = findFirstFieldMatching(value, "$config.Settings@", "::DEBUG");
		assertNotNull("$config.Settings@…::DEBUG must be not null", settingsDebug);
		assertNotNull("$_main::z must be not null", value.get("$__main__::z"));
		assertEquals("$__main__::x is not 3.", "\"3\"", value.get("$__main__::x").toString());
		assertEquals("$config.Settings@…::DEBUG is not true.", "\"true\"", settingsDebug.toString());
		assertEquals("$__main__::z is not true.", "\"true\"", value.get("$__main__::z").toString());
		assertNull("$config.settings should not be present, only $config::settings (probabaly, a problem with import.",
				type.get("$config.settings"));
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
