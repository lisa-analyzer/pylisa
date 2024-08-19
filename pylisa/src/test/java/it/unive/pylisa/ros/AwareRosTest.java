package it.unive.pylisa.ros;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import java.io.IOException;
import org.junit.Test;

public class AwareRosTest {

	@Test
	public void testAwareTalkNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/aware/talk_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("aware/aware-simple/talk_node"));
		lisa.run(program);
	}

	@Test
	public void testAwarePsychologistNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/aware/psychologist_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("aware/aware-simple/psychologist_node"));
		lisa.run(program);
	}

	@Test
	public void testAwareDirectorNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/aware/director_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("aware/aware-simple/director_node"));
		lisa.run(program);
	}

	@Test
	public void testAwareDialogNode() throws AnalysisSetupException, IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/aware/dialog_node.py",
				false);
		Program program = translator.toLiSAProgram();
		LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("aware/aware-simple/dialog_node"));
		lisa.run(program);
	}
}
