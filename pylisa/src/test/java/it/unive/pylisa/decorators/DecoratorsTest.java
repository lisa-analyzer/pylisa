package it.unive.pylisa.decorators;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import java.io.IOException;
import org.junit.Test;

public class DecoratorsTest {

	@Test
	public void test01() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/decorators/dec01.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("dec01");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testClassDecorators() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/decorators/class_decorator_test.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("class_decorators");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	public static LiSAConfiguration getLisaConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "tests/decorators/" + workdir;
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.optimize = false;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();

		InferredTypes type = new InferredTypes();

		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new SimpleAbstractDomain<>(heap, domain, type);
		return conf;
	}
}
