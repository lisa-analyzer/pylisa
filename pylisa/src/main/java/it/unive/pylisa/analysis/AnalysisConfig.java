package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.string.BoundedStringSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.outputs.HtmlResults;
import it.unive.lisa.outputs.JSONResults;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import java.util.function.Supplier;

/**
 * The analysis configurations pylisa runs with. They share
 * pylisa's field-sensitive heap and inferred Python types, and differ in the
 * value domain.
 * <p>
 * Every configuration is best-effort: calls to unknown code are assumed to
 * return an unknown value and to have no other effect, so results are not
 * sound over-approximations of every execution.
 * </p>
 */
public enum AnalysisConfig {

	/**
	 * Constant propagation: every value is either one known constant or
	 * unknown.
	 */
	CP(ConstantPropagation::new, ValueReader.constantPropagation()),

	/**
	 * Constant propagation together with a bounded set of strings: a string
	 * can also be known to be one of a few constants.
	 */
	CP_BSS(
			() -> new ValueDomainProduct<>(new ConstantPropagation(), new BoundedStringSet()),
			ValueReader.constantPropagationWithStringSets());

	/**
	 * The system property that, when {@code true}, makes the analyses also
	 * produce LiSA's HTML output, which shows the CFGs with the state of every
	 * statement.
	 */
	public static final String HTML_PROPERTY = "lisa.state.html";

	private final Supplier<ValueDomain<?>> valueDomain;

	private final ValueReader reader;

	AnalysisConfig(
			Supplier<ValueDomain<?>> valueDomain,
			ValueReader reader) {
		this.valueDomain = valueDomain;
		this.reader = reader;
	}

	/**
	 * Builds the LiSA configuration for this analysis, dumping the results
	 * under the given working directory only when asked to.
	 *
	 * @param workdir     the working directory, where LiSA writes its outputs
	 * @param dumpResults whether the results are dumped as JSON (and as HTML
	 *                        when the system property {@value #HTML_PROPERTY}
	 *                        is {@code true})
	 *
	 * @return the configuration
	 */
	LiSAConfiguration configuration(
			String workdir,
			boolean dumpResults) {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workdir;
		if (dumpResults) {
			conf.outputs.add(new JSONResults<>());
			if (Boolean.getBoolean(HTML_PROPERTY))
				conf.outputs.add(new HtmlResults<>(true));
		}
		configure(conf);
		return conf;
	}

	/**
	 * How many of the last call sites make up the context of a call. With
	 * two, a function called at one site inside a helper (such as a class
	 * constructor the helper calls) is analysed apart for each caller of the
	 * helper, instead of joining the states of all of them.
	 */
	public static final int CALL_STRING_DEPTH = 2;

	/**
	 * Sets the analysis of this configuration on a LiSA configuration: the
	 * interprocedural analysis, the call graph, the policy for unknown calls
	 * and the abstract domain. The working directory and the outputs are left
	 * to the caller, so that a subclass of {@link LiSAConfiguration}, such as
	 * the one LiSA's test executor takes, runs the same analysis.
	 *
	 * @param conf the configuration to set up
	 */
	public void configure(
			LiSAConfiguration conf) {
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>(CALL_STRING_DEPTH);
		conf.callGraph = new PyCallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.analysis = new SimpleAbstractDomain<>(
				new PyFieldSensitivePointBasedHeap(),
				valueDomain.get(),
				new PythonInferredTypes());
	}

	/**
	 * Yields the reader for the value domain of this configuration.
	 *
	 * @return the reader
	 */
	public ValueReader reader() {
		return reader;
	}

	/**
	 * Yields the label of this configuration, which also states that its
	 * results are best-effort.
	 *
	 * @return the label
	 */
	public String label() {
		return name() + "/best-effort";
	}
}
