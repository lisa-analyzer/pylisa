package it.unive.pylisa.decorators;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.ros.RosTestHelpers;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import org.junit.Test;

import java.io.IOException;

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
        var x = 3;
    }



    public static LiSAConfiguration getLisaConf(String workdir) {
        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "tests/decorators/" + workdir;
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;

        PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
                .bottom();
        TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());

        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
        return conf;
    }
}
