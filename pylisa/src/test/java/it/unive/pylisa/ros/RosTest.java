package it.unive.pylisa.ros;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataflow.rospropagation.RosTopic;
import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import org.junit.Test;

public class RosTest {

	@Test
	public void test() throws Exception {
		/*
		 * PyFrontend translator = new PyFrontend(
		 * "/Users/giacomozanatta/Projects/git-repos-downloader/repos/mechaship/mechaship_teleop/mechaship_teleop/mechaship_teleop_keyboard.py",
		 * false);
		 */
		PyFrontend translator = new PyFrontend(
				"ros-tests/action.py",
				false);
		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "ros-test-outputs/1-to-1-procedural-out";
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.NONE;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.optimize = false;
		// conf.openCallPolicy
		// conf.semanticChecks.add(new ROSComputationGraphDumper(new
		// RosComputationalGraph()));
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		// conf.interproceduralAnalysis = new ContextBasedAnalysis();
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.semanticChecks.add(new ROSComputationGraphDumper(new RosComputationalGraph(), new ROSNetwork()));
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);

		ROSApplication r = new RosApplicationBuilder()
				.withNode(
						new PythonROSNodeBuilder(
								"ros-tests/action.py"))
				.withWorkDir("ros-test-outputs/out-test.py").build();
		r.dumpGraph();

	}

	@Test
	public void testConstant() throws Exception {
		PyFrontend translator = new PyFrontend("ros-tests/constant-prop.py", false);

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "ros-test-outputs/test-ros-output-constant";
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		// conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		// conf.openCallPolicy
		RosTopic rt = new RosTopic();
		conf.optimize = false;
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		ConstantPropagation constantPropagation = new ConstantPropagation();
		// conf.semanticChecks.add(new
		// it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testLoopExpr() throws Exception {
		PyFrontend translator = new PyFrontend("ros-tests/test_flow.py", false);

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "ros-test-outputs/test-ros-output-flow";
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		// conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		// conf.openCallPolicy
		RosTopic rt = new RosTopic();
		conf.optimize = false;
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		ConstantPropagation constantPropagation = new ConstantPropagation();
		// conf.semanticChecks.add(new
		// it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testTypedArgs() throws Exception {
		PyFrontend translator = new PyFrontend("ros-tests/typedargs/typed.py", false);

		Program program = translator.toLiSAProgram();

		new LiSA(RosTestHelpers.getLisaConf("typedargs/typed.py")).run(program);
	}

	@Test
	public void testSet() throws Exception {
		PyFrontend translator = new PyFrontend("../test.py", false);

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "ros-test-outputs/test-ros-output-set";
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		// conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		// conf.openCallPolicy
		RosTopic rt = new RosTopic();
		conf.optimize = false;
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		ConstantPropagation constantPropagation = new ConstantPropagation();
		// conf.semanticChecks.add(new
		// it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testTwoFiles() throws Exception {
		PyFrontend translator1 = new PyFrontend("ros-tests/two-nodes/first_file.py", false);
		Program program1 = translator1.toLiSAProgram();
		PyFrontend translator2 = new PyFrontend("ros-tests/two-nodes/second_file.py", false);
		Program program2 = translator2.toLiSAProgram();
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "ros-test-outputs/two-files";
		conf.serializeResults = true;
		conf.jsonOutput = true;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		// conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		// conf.openCallPolicy
		conf.optimize = false;
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		ConstantPropagation constantPropagation = new ConstantPropagation();
		// conf.semanticChecks.add(new
		// it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		LiSA lisa = new LiSA(conf);
		lisa.run(program1, program2);
	}
}
