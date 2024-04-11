package it.unive.pylisa.fastapi;

import  it.unive.lisa.LiSA;
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
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.checks.FastApiHalfwaySemanticChecker;
import it.unive.pylisa.checks.FastApiSyntacticChecker;
import org.junit.Test;

import java.io.IOException;

public class TestGround {

    @Test
    public void initial() throws IOException {

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-outputs/";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;
        conf.syntacticChecks.add(new FastApiSyntacticChecker());
//        conf.semanticChecks.add(new FastApiHalfwaySemanticChecker<>());

        PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap().bottom();
        TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);

        PyFrontend frontend1 = new PyFrontend(
                "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_a.py",
                false);

//        PyFrontend frontend2 = new PyFrontend(
//                "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_b.py",
//                false);

        Program program1 = frontend1.toLiSAProgram();
//        Program program2 = frontend2.toLiSAProgram();
        LiSA lisa = new LiSA(conf);

        try {
//            lisa.run(program1, program2);
            lisa.run(program1);
        } catch (Exception ignored) { }
    }
}
