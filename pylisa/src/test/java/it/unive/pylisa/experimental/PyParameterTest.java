package it.unive.pylisa.experimental;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import java.io.IOException;
import org.junit.Test;

public class PyParameterTest {

	@Test
	public void test1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/typehints/th01.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("tests/");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	public static LiSAConfiguration getLisaConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "outputs/" + workdir;
		conf.outputs.add(new JSONResults<>());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();

		InferredTypes type = new InferredTypes();

		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new SimpleAbstractDomain<>(heap, domain, type);
		return conf;
	}
}
