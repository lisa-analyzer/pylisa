package it.unive.pylisa.helpers;

import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.outputs.HtmlInputs;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.program.cfg.fixpoints.optforward.OptimizedForwardAscendingFixpoint;
import it.unive.pylisa.analysis.PyReachability;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.constants.ConstantPropagationDomain;
import it.unive.pylisa.analysis.heap.PyHeap;
import it.unive.pylisa.analysis.types.PyInferredTypes;
import it.unive.pylisa.checks.ExceptionChecker;

public class TestHelper {

	public static CronConfiguration cfgConfig() {
		CronConfiguration conf = new CronConfiguration();
		conf.optimize = true;
		conf.forwardFixpoint = new OptimizedForwardAscendingFixpoint<>();
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new HtmlInputs(true));
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		return conf;
	}

	/**
	 * Builds a configuration that runs a full analysis using integer constant
	 * propagation as the value domain, paired with a field-sensitive
	 * point-based heap domain and type inference.
	 */
	public static CronConfiguration constantPropagationConfig() {
		CronConfiguration conf = new CronConfiguration();
		conf.optimize = false;
		conf.outputs.add(new JSONResults<>());
//		conf.forceUpdate = true;
//		conf.outputs.add(new HtmlResults<>(true));
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		// as in the command line interface (see PyLiSA), and in JLiSA
		conf.analysis = new PyReachability<>(new SimpleAbstractDomain<HeapEnvWithFields,
				ValueEnvironment<ConstantPropagation>, TypeEnvironment<TypeSet>>(
						new PyHeap(), new ConstantPropagationDomain(), new PyInferredTypes()));

		return conf;
	}

	/**
	 * Builds the configuration of {@link #constantPropagationConfig()}, also
	 * checking the properties of the SV-COMP benchmarks (see
	 * {@link ExceptionChecker}).
	 */
	public static CronConfiguration svcompConfig() {
		CronConfiguration conf = constantPropagationConfig();
		conf.semanticChecks.add(new ExceptionChecker<>());
		return conf;
	}
}
