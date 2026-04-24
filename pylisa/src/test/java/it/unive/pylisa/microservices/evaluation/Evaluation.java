package it.unive.pylisa.microservices.evaluation;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.network.NetworkAwareAbstractDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.listeners.BottomTopListener;
import it.unive.lisa.listeners.TracingListener;
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.interprocedural.NetworkAwareContextBasedAnalysis;
import it.unive.pylisa.outputs.FinalNetworkMermaidResults;
import it.unive.pylisa.outputs.FinalNetworkTxtResults;
import it.unive.pylisa.outputs.MermaidNetworkResults;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;

public class Evaluation {

	/**
	 * Root folder under which ground-truth {@code final-network.txt} files are
	 * checked in, one per evaluation test. The directory ships with the test
	 * sources so it is not subject to the {@code tests/} gitignore rule.
	 */
	private static final Path GROUND_TRUTH_ROOT = Paths.get("src/test/resources/ground-truth");

	/**
	 * Runs LiSA with the given configuration and then asserts that the
	 * generated {@code final-network.txt} matches the checked-in ground truth.
	 * <p>
	 * The expected path is derived from {@code conf.workdir} by stripping the
	 * leading {@code tests/} segment; e.g. a workdir of
	 * {@code tests/microservices/evaluation/FastVector} yields
	 * {@code src/test/resources/ground-truth/microservices/evaluation/FastVector/final-network.txt}.
	 * <p>
	 * Set {@code -Dlisa.cron.update=true} to seed a missing ground-truth or
	 * regenerate one whose output has intentionally changed.
	 */
	private static void runAndAssertNetworkTxt(
			LiSAConfiguration conf,
			Program program)
			throws IOException {
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
		Path actual = Paths.get(conf.workdir, "final-network.txt");
		String suffix = conf.workdir.startsWith("tests/") ? conf.workdir.substring("tests/".length())
				: conf.workdir;
		Path expected = GROUND_TRUTH_ROOT.resolve(suffix).resolve("final-network.txt");
		FinalNetworkTxtComparer.assertMatches(expected, actual);
	}

	@Test
	public void FastVector() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/microservices/evaluation/FastVector/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/FastVector");
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void TestHexagonalArchitecture() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/microservices/evaluation/hexagonal-architecture-python/src/app.py",
				false,
				"py-testcases/microservices/evaluation/hexagonal-architecture-python");
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/hexagonal-architecture-python");
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void testAstrobase() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/microservices/evaluation/astrobase/astrobasecloud/server/main.py",
				false,
				"py-testcases/microservices/evaluation/astrobase");
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/astrobase");
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void testNetflixDispatch() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/microservices/evaluation/dispatch/src/dispatch/main.py",
				false,
				"py-testcases/microservices/evaluation/dispatch/src/dispatch");
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/dispatch");
		conf.asynchronousListeners.add(new TracingListener(TracingListener.TraceLevel.ALL));
		conf.asynchronousListeners.add(new BottomTopListener());
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void testFastAPIEcommerce() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/microservices/evaluation/E-commerce-FastAPI/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/E-commerce-FastAPI");
		conf.outputs.add(new FinalNetworkMermaidResults<>());
		conf.outputs.add(new MermaidNetworkResults<>(false));
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void testROS2_RMW() throws IOException {
		PyFrontend translator = new PyFrontend(
				"/Users/giacomo/02-work/01-ros2-rmw/src/rmw_firewall_web/rmw_firewall_web/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/rmw_firewall_web");
		runAndAssertNetworkTxt(conf, program);
	}

	@Test
	public void testROS2_RMW_ca_prov() throws IOException {
		PyFrontend translator = new PyFrontend(
				"/Users/giacomo/02-work/01-ros2-rmw/src/rmw_security_provisioner/rmw_security_provisioner/main.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices/evaluation/rmw_security_provisioner");
		runAndAssertNetworkTxt(conf, program);
	}

	public static LiSAConfiguration getLisaConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "tests/" + workdir;
		// conf.outputs.add(new JSONResults<>());
		// conf.outputs.add(new HtmlResults<>(false));
		// conf.outputs.add(new MermaidNetworkResults<>(false));
		// conf.outputs.add(new ApplicationStructure());
		// conf.outputs.add(new FinalNetworkMermaidResults<>());
		conf.outputs.add(new FinalNetworkTxtResults<>());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		PyFieldSensitivePointBasedHeap heap = new PyFieldSensitivePointBasedHeap();

		PythonInferredTypes type = new PythonInferredTypes();
		// PythonValueDomain<ConstantPropagation> valueDomain = new
		// PythonValueDomain<ConstantPropagation>(new ConstantPropagation());
		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new NetworkAwareAbstractDomain<>(
				new SimpleAbstractDomain<>(heap, domain, type),
				NetworkAwareContextBasedAnalysis.pyFunctionNameExtractor());
		return conf;
	}

	@Test
	public void functions_redecl() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/functions_redeclaration.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("functions_redeclaration");
		// conf.outputs.add(new FinalNetworkMermaidResults<>());
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}
}