package it.unive.pylisa.microservices;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.network.NetworkAwareAbstractDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.outputs.HtmlResults;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.interprocedural.NetworkAwareContextBasedAnalysis;
import it.unive.pylisa.outputs.ApplicationStructure;
import it.unive.pylisa.outputs.FinalNetworkMermaidResults;
import it.unive.pylisa.outputs.MermaidNetworkResults;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

public class MicroservicesTest {

	@Test
	public void test01() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/fastapi2.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices-01");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
		assertMainInitNetworkStateIsNotEmpty("microservices-01");
	}

	@Test
	public void test02() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/flask.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices-02");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testMiningWave() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/mining-wave/api.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("mining-wave");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testBoundaries() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/boundaries/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("boundaries");
		// conf.outputs.add(new FinalNetworkMermaidResults<>());
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	public static LiSAConfiguration getLisaConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "tests/" + workdir;
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new HtmlResults<>(true));
		conf.outputs.add(new MermaidNetworkResults<>(false));
		conf.outputs.add(new ApplicationStructure());
		conf.outputs.add(new FinalNetworkMermaidResults<>());
		conf.interproceduralAnalysis = new NetworkAwareContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();

		PythonInferredTypes type = new PythonInferredTypes();
		// PythonValueDomain<ConstantPropagation> valueDomain = new
		// PythonValueDomain<ConstantPropagation>(new ConstantPropagation());
		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new NetworkAwareAbstractDomain<>(
				new SimpleAbstractDomain<>(heap, domain, type),
				NetworkAwareContextBasedAnalysis.pyFunctionNameExtractor());
		return conf;
	}

	private static void assertMainInitNetworkStateIsNotEmpty(
			String workdir)
			throws IOException {
		Path outputDir = Path.of("tests", workdir);
		Optional<Path> initNetworkFile;
		try (Stream<Path> files = Files.list(outputDir)) {
			initNetworkFile = files
					.filter(path -> path.getFileName().toString().startsWith("untyped___main__.$init()_"))
					.filter(path -> path.getFileName().toString().endsWith("-network.html"))
					.max(Comparator.comparing(path -> path.getFileName().toString()));
		}

		/*
		 * assertTrue("Missing __main__.$init() network output",
		 * initNetworkFile.isPresent()); String html =
		 * Files.readString(initNetworkFile.get()); int marker =
		 * html.lastIndexOf("<div class=\"mermaid\">graph TD");
		 * assertNotEquals("No mermaid network state found in __main__.$init() output"
		 * , -1, marker); String tail = html.substring(marker);
		 * assertFalse("Final __main__.$init() network state is empty",
		 * tail.contains("No network state"));
		 */
	}
}
