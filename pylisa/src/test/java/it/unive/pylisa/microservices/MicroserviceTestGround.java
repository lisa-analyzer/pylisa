package it.unive.pylisa.microservices;

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
import it.unive.pylisa.checks.FastApiSyntacticChecker;
import it.unive.pylisa.libraries.fastapi.graph.EndpointGraphBuilder;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;

public class MicroserviceTestGround {

    private LiSAConfiguration conf;
    private final FastApiSyntacticChecker syntacticChecker = new FastApiSyntacticChecker();
    private final EndpointGraphBuilder graphBuilder = new EndpointGraphBuilder();

    @Before
    public void before() {

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-outputs/";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;
        conf.syntacticChecks.add(syntacticChecker);

        PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap().bottom();
        TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);

        this.conf = conf;
    }

    @Test
    public void twoFiles() throws IOException {

        PyFrontend frontend1 = new PyFrontend(
                "py-testcases/microservices/basic/microservice_a.py",
                false);

        PyFrontend frontend2 = new PyFrontend(
                "py-testcases/microservices/basic/microservice_b.py",
                false);

        Program program1 = frontend1.toLiSAProgram();
        LiSA lisa = new LiSA(this.conf);
        lisa.run(program1);

        Program program2 = frontend2.toLiSAProgram();
        LiSA lisa2 = new LiSA(this.conf);
        lisa2.run(program2);

        graphBuilder.build(syntacticChecker.endpoints);
    }

    @Test
    public void threeFiles() throws IOException {

        PyFrontend frontend1 = new PyFrontend(
                "py-testcases/microservices/level-1-complexity/microservice_a.py",
                false);

        PyFrontend frontend2 = new PyFrontend(
                "py-testcases/microservices/level-1-complexity/microservice_b.py",
                false);

        PyFrontend frontend3 = new PyFrontend(
                "py-testcases/microservices/level-1-complexity/microservice_c.py",
                false);

        LiSA lisa = new LiSA(this.conf);
        Program program1 = frontend1.toLiSAProgram();
        lisa.run(program1);

        LiSA lisa2 = new LiSA(this.conf);
        Program program2 = frontend2.toLiSAProgram();
        lisa2.run(program2);

        LiSA lisa3 = new LiSA(this.conf);
        Program program3 = frontend3.toLiSAProgram();
        lisa3.run(program3);

        graphBuilder.build(syntacticChecker.endpoints);
    }

    @Test
    public void withoutDecorators() throws IOException {

        // TODO: Adapt this test case to the latest changes.
        PyFrontend frontend1 = new PyFrontend(
                "py-testcases/microservices/undecorated/nodec_microservice_a.py",
                false);

        Program program1 = frontend1.toLiSAProgram();
        LiSA lisa = new LiSA(this.conf);

        lisa.run(program1);
    }
}
