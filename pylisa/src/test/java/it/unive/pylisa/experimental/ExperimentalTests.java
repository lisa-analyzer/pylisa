package it.unive.pylisa.experimental;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.ros.RosTestHelpers;
import org.junit.Test;

import java.io.IOException;

public class ExperimentalTests {
    @Test
    public void test1() throws IOException {
        PyFrontend translator = new PyFrontend(
                "ros-tests/simple_node/one_publisher.py",
                false);
        Program program1 = translator.toLiSAProgram(false);
        LiSAConfiguration conf = RosTestHelpers.getLisaConf("ros-tests/simple_node/one_publisher");
        LiSA lisa = new LiSA(conf);
        PyFrontend translator1 = new PyFrontend(
                "ros-tests/simple_node/action.py",
                false);
        Program program2 = translator1.toLiSAProgram(false);
        lisa.run(program1, program2);
        //lisa.run(program2);
    }


    @Test
    public void testPika() throws IOException {

    }
}
