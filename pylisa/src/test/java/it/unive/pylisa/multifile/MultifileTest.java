package it.unive.pylisa.multifile;

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
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import org.junit.Test;

import java.io.IOException;

public class MultifileTest {

    @Test
    public void test01() throws IOException {
        PyFrontend translator = new PyFrontend(
                "py-testcases/multi-file/prog01/main.py",
                false);
        Program program = translator.toLiSAProgram(false);
        LiSAConfiguration conf = getLisaConf("prog01");
        LiSA lisa = new LiSA(conf);
        var t = lisa.run(program);
        var x = 3;
    }



    public static LiSAConfiguration getLisaConf(String workdir) {
        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "tests/multi-file/" + workdir;
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
