package it.unive.pylisa.ros;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import java.io.IOException;
import org.junit.Test;

public class ElephantShooterTest {

	@Test
	public void testApplication() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
		new RosApplicationBuilder().withNode(new PythonROSNodeBuilder("ros-tests/elephant_shooter/mpc_node.py"))
				.withWorkDir("ros-test-outputs/elephant-shooter/shooter_test")
				.build().dumpGraph();
	}

	@Test
	public void testFOpenParen() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
		new RosApplicationBuilder().withNode(new PythonROSNodeBuilder("ros-tests/elephant_shooter/test_open_paren.py"))
				.withWorkDir("ros-test-outputs/elephant-shooter/test_open_paren")
				.build().dumpGraph();
	}

	@Test
	public void testCollectorNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/fruit_collectors/collector_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("fruit-collectors/simple/collector_node"));
		lisa.run(program);
	}

	@Test
	public void testVisionNodeROSApp() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
		new RosApplicationBuilder().withNode(new PythonROSNodeBuilder("ros-tests/fruit_collectors/vision_node.py"))
				.withWorkDir("ros-test-outputs/fruit-collectors/simple-rosApp/vision_node").build().dumpGraph();
	}

	@Test
	public void testCollectorNodeROSApp() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
		new RosApplicationBuilder().withNode(new PythonROSNodeBuilder("ros-tests/fruit_collectors/collector_node.py"))
				.withWorkDir("ros-test-outputs/fruit-collectors/simple-rosApp/collector_node").build().dumpGraph();
	}

	@Test
	public void testVisionNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/fruit_collectors/vision_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("fruit-collectors/simple-rosApp/vision_node"));
		lisa.run(program);
	}
}
