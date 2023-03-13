package it.unive.pylisa.ros;

import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataflow.rospropagation.RosTopic;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.checks.RosTopicDeclarationFinder;
import org.junit.Test;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

public class RosTest {


    @Test
    public void test() throws Exception {
        PyFrontend translator = new PyFrontend("ros-tests/main.py", false);

        Program program = translator.toLiSAProgram();

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-ros-output";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        //conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        //conf.openCallPolicy
        RosTopic rt = new RosTopic();
        conf.syntacticChecks.add(new RosTopicDeclarationFinder());
        //conf.semanticChecks.add(new DataframeDumper(conf));
        //conf.semanticChecks.add(new BottomFinder<>());
        //conf.semanticChecks.add(new DataframeStructureConstructor());
        PointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
        InferredTypes type = new InferredTypes();
        //conf.interproceduralAnalysis = new ContextBasedAnalysis();
        DataframeGraphDomain domain = new DataframeGraphDomain();
        conf.abstractState = getDefaultFor(AbstractState.class, heap, domain, type);
        LiSA lisa = new LiSA(conf);
        lisa.run(program);
    }
}