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
import it.unive.pylisa.libraries.fastapi.EndpointGraphBuilder;
import it.unive.pylisa.libraries.fastapi.models.Endpoint;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.util.List;

import static guru.nidi.graphviz.model.Factory.graph;
import static guru.nidi.graphviz.model.Factory.node;
import static guru.nidi.graphviz.model.Link.to;

public class MicroserviceTestGround {

    private LiSAConfiguration conf;

    private List<Endpoint> endpoints;
    private final FastApiSyntacticChecker syntacticChecker = new FastApiSyntacticChecker();


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
    public void initial() throws IOException {

        PyFrontend frontend = new PyFrontend(
                "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_a.py",
                false);

        Program program1 = frontend.toLiSAProgram();
        LiSA lisa = new LiSA(this.conf);

        lisa.run(program1);

        endpoints = syntacticChecker.endpoints;

        EndpointGraphBuilder.build(endpoints);
    }
}
