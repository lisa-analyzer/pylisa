package it.unive.pylisa.imports;

import static it.unive.pylisa.testutil.LiSAConfigs.getDefaultConf;
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
		LiSAConfiguration conf = getDefaultConf("imports/import2");
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
		LiSAConfiguration conf = getDefaultConf("imports/import3");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");

		assertImports("imports/import3");
	}

	// imports4() lived here previously: it tests that a multi-file
	// `from routes import miningcore` chain resolves miningcore.router to
	// fastapi.APIRouter*. That assertion needs fastapi.txt + the FastAPI
	// Java backends, which moved to lisa-network as part of the
	// pylisa<->lisa-network decoupling. The full-resolution test now lives
	// at it.unive.lisa.microservices.imports.ImportsTest.imports4().
	//
	// FOLLOW-UP: re-add a pylisa-side imports4 that asserts the degraded
	// fallback when fastapi is *not* on the classpath: the open-call
	// fallback should make `miningcore.router` resolve to TOP / Unknown
	// Module, while `z = 3` still resolves to the constant "3". This
	// verifies the type system stays sound under missing-library fallback.

	@Test
	public void imports1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/imports/import1/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getDefaultConf("imports/import1");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull(program.getUnit("builtins.object.__new__"), "missing __new__ builtins.object.__new__.");
		assertNotNull(program.getUnit("builtins.object.__init__"), "missing __init__ builtins.object.__init__.");
		assertNotNull(program.getUnit("builtins.object.super"), "missing super builtins.object.super.");

		assertImports("imports/import1");
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
		JsonNode exitAnalysisState = root.get("descriptions").get(nodesCount - 1).get("description").get("state");
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
