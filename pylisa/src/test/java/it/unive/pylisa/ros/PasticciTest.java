package it.unive.pylisa.ros;

import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.ros.models.rclpy.ROSNetwork;
import org.junit.Test;

public class PasticciTest {

    @Test
    public void Test01() throws Exception {
        RosApplicationBuilder rob = new RosApplicationBuilder();
        try {
            //rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/pasticcio02.py"));
            rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/pasticcio02.py"));
        } catch (ROSNodeBuildException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        rob.withWorkDir("ros-test-outputs/pasticci/30012024-1");

        ROSApplication ra = rob.build();
        ROSNetwork n = ra.getRosNetwork();
        ra.dumpResults();
    }
}