package it.unive.pylisa.helpers;

import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration.GraphType;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.pylisa.analysis.constants.ConstantPropagation;

public class TestHelper {

	public static CronConfiguration cfgConfig() {
		CronConfiguration conf = new CronConfiguration();
		conf.optimize = true;
		conf.jsonOutput = true;
		conf.serializeInputs = true;
		conf.analysisGraphs = GraphType.HTML_WITH_SUBNODES;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		return conf;
	}

	/**
	 * Builds a configuration that runs a full analysis using integer
	 * constant propagation as the value domain, paired with a
	 * field-sensitive point-based heap domain and type inference.
	 */
	public static CronConfiguration constantPropagationConfig() {
		CronConfiguration conf = new CronConfiguration();
		conf.optimize = false;
		conf.jsonOutput = true;
		conf.serializeResults = true;
//		conf.forceUpdate = true;
//		conf.analysisGraphs = GraphType.HTML_WITH_SUBNODES;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		ValueEnvironment<ConstantPropagation> constants = new ValueEnvironment<>(
				new ConstantPropagation());
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.abstractState = new SimpleAbstractState<>(heap, constants, type);

		return conf;
	}
}
