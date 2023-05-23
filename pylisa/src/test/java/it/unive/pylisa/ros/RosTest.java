package it.unive.pylisa.ros;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.callgraph.CHACallGraph;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
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
        PyFrontend translator = new PyFrontend("ros-tests/main2.py", false);

        Program program = translator.toLiSAProgram();

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-ros-output";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        //conf.openCallPolicy
        RosTopic rt = new RosTopic();
        // conf.syntacticChecks.add(new ROSComputationGraphDumper());
        //conf.semanticChecks.add(new DataframeDumper(conf));
        //conf.semanticChecks.add(new BottomFinder<>());
        //conf.semanticChecks.add(new DataframeStructureConstructor());
        PointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
        InferredTypes type = new InferredTypes();
        //conf.interproceduralAnalysis = new ContextBasedAnalysis();
        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<ConstantPropagation>(new ConstantPropagation());
        conf.semanticChecks.add(new ROSComputationGraphDumper());
        conf.abstractState = getDefaultFor(AbstractState.class, heap, domain, type);
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

        PointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
        InferredTypes type = new InferredTypes();
        //conf.interproceduralAnalysis = new ContextBasedAnalysis();
        ConstantPropagation constantPropagation = new ConstantPropagation();
        conf.semanticChecks.add(new it.unive.pylisa.checks.semantics.RosTopicDeclarationFinder());
        conf.abstractState = getDefaultFor(AbstractState.class, heap, constantPropagation, type);
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }
}
