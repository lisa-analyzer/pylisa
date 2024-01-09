package it.unive.pylisa.pika;

import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.PikaChecker;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.pika.PikaNetwork;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.network.Network;

public class PikaTestHelpers {
    public static LiSAConfiguration getLisaConf(String workdir) {
        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "pika-test-outputs/" + workdir;
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
        //RosComputationalGraph graph = new RosComputationalGraph();
        conf.semanticChecks.add(new PikaChecker(new PikaNetwork()));
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
        return conf;
    }

}
