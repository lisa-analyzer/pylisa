package it.unive.pylisa.experimental;

import java.io.IOException;

import org.junit.Test;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.RosComputationalGraph;

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
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.optimize = false;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap().bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());

		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		RosComputationalGraph graph = new RosComputationalGraph();
		conf.semanticChecks.add(new ROSComputationGraphDumper(graph, new ROSNetwork()));
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		return conf;
	}
}
