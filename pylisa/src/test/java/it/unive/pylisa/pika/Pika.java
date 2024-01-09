package it.unive.pylisa.pika;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.ros.RosTestHelpers;
import it.unive.ros.lisa.checks.semantics.PikaChecker;
import it.unive.ros.network.Network;
import org.junit.Test;

import java.io.IOException;

public class Pika {

    @Test
    public void test() throws Exception {
        PyFrontend translator1 = new PyFrontend(
                "pika_tests/amqp1.py",
                false);
        Program program1 = translator1.toLiSAProgram();
        PyFrontend translator2 = new PyFrontend(
                "pika_tests/amqp2.py",
                false);
        Program program2 = translator2.toLiSAProgram();
        LiSAConfiguration conf = PikaTestHelpers.getLisaConf("pika-tests/amqp");
        LiSA lisa = new LiSA(conf);
        PikaChecker checker = (PikaChecker) conf.semanticChecks.stream().iterator().next();

        Network pikaNetwork = checker.getNetwork();
        lisa.run(program1);
        lisa.run(program2);

        pikaNetwork.processEvents();
    }

}
