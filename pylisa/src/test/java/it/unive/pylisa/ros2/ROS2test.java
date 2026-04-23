package it.unive.pylisa.ros2;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;

import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.io.IOException;
import org.junit.Test;

public class ROS2test {

	@Test
	public void testROS2_1() throws IOException {
		PyFrontend translator = new PyFrontend(
				"ros-tests/fruit_collectors/vision_node.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("microservices-02");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}
}
