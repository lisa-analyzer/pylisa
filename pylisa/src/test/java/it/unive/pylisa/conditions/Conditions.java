package it.unive.pylisa.conditions;

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

public class Conditions {
	@Test
	public void conditions1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/conditions/condition1/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("conditions/condition1");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		assertNotNull("missing __new__ builtins.object.__new__.", program.getUnit("builtins.object.__new__"));
		assertNotNull("missing __init__ builtins.object.__init__.", program.getUnit("builtins.object.__init__"));
		assertNotNull("missing super builtins.object.super.", program.getUnit("builtins.object.super"));

		assertConditions1("conditions/condition1");
	}

	private void assertConditions1(
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
		System.out.println("[V] $__main__::x == \"3\"");
		assertNull(
				"$config.settings should not be present, only $config::settings (probabaly, a problem with from config import settings.",
				type.get("$config.settings"));
		assertEquals("\"3\"", value.get("$__main__::x").toString());
	}
}
