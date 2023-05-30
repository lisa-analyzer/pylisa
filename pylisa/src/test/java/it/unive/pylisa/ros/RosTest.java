package it.unive.pylisa.ros;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.traces.TracePartitioning;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.CHACallGraph;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.FieldSensitivePointBasedHeapWithConvAs;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataflow.rospropagation.RosTopic;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.checks.RosTopicDeclarationFinder;
import it.unive.pylisa.checks.semantics.ROSComputationGraphDumper;
import org.junit.Test;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

public class RosTest {


    @Test
    public void test() throws Exception {
        PyFrontend translator = new PyFrontend("ros-tests/main3.py", false);

        Program program = translator.toLiSAProgram();

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-ros-output-3";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;
        //conf.openCallPolicy
        conf.semanticChecks.add(new ROSComputationGraphDumper());
        FieldSensitivePointBasedHeapWithConvAs heap = new FieldSensitivePointBasedHeapWithConvAs();
        TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
        //conf.interproceduralAnalysis = new ContextBasedAnalysis();
        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
        //conf.semanticChecks.add(new ROSComputationGraphDumper());
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }
    @Test
    public void testConstant() throws Exception {
        PyFrontend translator = new PyFrontend("ros-tests/constant-prop.py", false);

        Program program = translator.toLiSAProgram();

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-ros-output-constant";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        //conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        //conf.openCallPolicy
        RosTopic rt = new RosTopic();

        PyFieldSensitivePointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
        InferredTypes type = new InferredTypes();
        //conf.interproceduralAnalysis = new ContextBasedAnalysis();
        ConstantPropagation constantPropagation = new ConstantPropagation();
        //conf.semanticChecks.add(new it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
        conf.abstractState = getDefaultFor(AbstractState.class, heap, constantPropagation, type);
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }
}
