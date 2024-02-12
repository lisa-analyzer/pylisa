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
            rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/pasticcio04.py"));
        } catch (ROSNodeBuildException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        rob.withWorkDir("ros-test-outputs/pasticci/01022024-1");

        ROSApplication ra = rob.build();
        ROSNetwork n = ra.getRosNetwork();
        n.processEvents();
        ra.dumpResults();
        System.out.println(n.toMermaid());
    }

    @Test
    public void TestIntelPub() throws Exception {
        RosApplicationBuilder rob = new RosApplicationBuilder();
        try {
            //rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/pasticcio02.py"));
            rob.withNode(new PythonROSNodeBuilder("ros-tests/lasagna/publisher_local_function.py"));
        } catch (ROSNodeBuildException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        rob.withWorkDir("ros-test-outputs/pasticci/qos");

        ROSApplication ra = rob.build();
        ROSNetwork n = ra.getRosNetwork();
        n.processEvents();
        ra.dumpResults();
        System.out.println(n.toMermaid());
        System.out.println(n.toGraphviz(false));
    }
}