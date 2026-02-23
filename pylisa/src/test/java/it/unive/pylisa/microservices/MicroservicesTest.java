package it.unive.pylisa.microservices;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.type.TypeDomain;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.lattices.ConstantValue;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import java.io.IOException;

import it.unive.lisa.analysis.network.NetworkAwareAbstractDomain;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import org.junit.Test;

public class MicroservicesTest {

    @Test
    public void test01() throws IOException {
        PyFrontend translator = new PyFrontend(
                "py-testcases/fastapi2.py",
                false);
        Program program = translator.toLiSAProgram(false);
        LiSAConfiguration conf = getLisaConf("microservices-01");
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }

    @Test
    public void test02() throws IOException {
        PyFrontend translator = new PyFrontend(
                "py-testcases/flask.py",
                false);
        Program program = translator.toLiSAProgram(false);
        LiSAConfiguration conf = getLisaConf("microservices-02");
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }

    public static LiSAConfiguration getLisaConf(
            String workdir) {
        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "tests/" + workdir;
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;

        FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();

        PythonInferredTypes type = new PythonInferredTypes();
        //PythonValueDomain<ConstantPropagation> valueDomain = new PythonValueDomain<ConstantPropagation>(new ConstantPropagation());
        ConstantPropagation domain = new ConstantPropagation();
        conf.analysis = new NetworkAwareAbstractDomain<>(new SimpleAbstractDomain<>(heap, domain, type));
        return conf;
    }
}
