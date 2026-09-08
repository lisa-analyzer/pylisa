package it.unive.pylisa.helpers;

import it.unive.lisa.conf.LiSAConfiguration.GraphType;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;

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
}
