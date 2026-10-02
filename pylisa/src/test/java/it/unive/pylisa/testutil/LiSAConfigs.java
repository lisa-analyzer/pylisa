package it.unive.pylisa.testutil;

import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.outputs.HtmlResults;
import it.unive.lisa.outputs.JSONResults;
import it.unive.pylisa.analysis.PyCallGraph;
import it.unive.pylisa.analysis.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;

/**
 * Shared {@link LiSAConfiguration} factory for non-network pylisa tests.
 * Replaces the historical use of {@code MicroservicesTest.getLisaConf} from
 * {@code pylisa.microservices} (which lived in pylisa before the
 * network-decoupling refactor and got moved to lisa-network).
 */
public final class LiSAConfigs {

	private LiSAConfigs() {
	}

	/**
	 * Returns a basic LiSA configuration with HTML + JSON outputs, RTA call
	 * graph, context-based interprocedural analysis, and a constant-propagation
	 * value domain over a field-sensitive point-based heap. No network
	 * outputs, no network-aware analysis.
	 *
	 * @param workdir the relative workdir under {@code tests/}
	 *
	 * @return the configuration
	 */
	public static LiSAConfiguration getDefaultConf(
			String workdir) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = "tests/" + workdir;
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new HtmlResults<>(true));
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new PyCallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		PyFieldSensitivePointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
		PythonInferredTypes type = new PythonInferredTypes();
		ConstantPropagation domain = new ConstantPropagation();
		conf.analysis = new SimpleAbstractDomain<>(heap, domain, type);
		return conf;
	}
}
