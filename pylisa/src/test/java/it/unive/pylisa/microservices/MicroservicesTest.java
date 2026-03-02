package it.unive.pylisa.microservices;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.network.NetworkAwareAbstractDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.outputs.HtmlResults;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import it.unive.pylisa.outputs.MermaidNetworkResults;
import java.io.IOException;
import org.junit.Test;

public class MicroservicesTest {

	@Test
	public void test01() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/fastapi2.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("microservices-01");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void test02() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/flask.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("microservices-02");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	public static LiSAConfiguration getLisaConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "tests/" + workdir;
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new HtmlResults<>(false));
		conf.outputs.add(new MermaidNetworkResults<>(false));
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();

		PythonInferredTypes type = new PythonInferredTypes();
		// PythonValueDomain<ConstantPropagation> valueDomain = new
		// PythonValueDomain<ConstantPropagation>(new ConstantPropagation());
		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new NetworkAwareAbstractDomain<>(new SimpleAbstractDomain<>(heap, domain, type));
		return conf;
	}
}
