package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that the call graph lists every Python call that may reach a
 * callable among its call sites, however the callee was obtained, and each
 * call once.
 */
class CallSitesTest {

	private static final String DIR = "src/test/resources/callgraph/sites/";

	private static final String ECHO = "testnatives.echo::$call";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void libraryModelCalledInFourWaysHasFourSites(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "four_ways.py", config);
		Collection<Call> sites = run.sitesOf(ECHO);
		assertTrue(sites.stream().allMatch(PyCall.class::isInstance), sites.toString());
		assertEquals(Set.of(run.lineOf("@module"), run.lineOf("@variable"), run.lineOf("@imported"),
				run.lineOf("@again")), lines(sites));
		assertEquals(4, sites.size(), "each call is listed once: " + sites);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void methodCalledThroughAVariableIsASite(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "bound_method.py", config);
		Collection<Call> sites = run.sitesEndingWith(".greet::$call");
		assertTrue(sites.stream().allMatch(PyCall.class::isInstance), sites.toString());
		assertEquals(Set.of(run.lineOf("@direct"), run.lineOf("@through")), lines(sites));
		assertEquals(2, sites.size(), "each call is listed once: " + sites);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void libraryModelMethodCalledOnAnObjectHasItsSite(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "object_method.py", config);
		Collection<Call> sites = run.sitesOf("testnatives.Plain.echo::$call");
		assertTrue(sites.stream().allMatch(PyCall.class::isInstance), sites.toString());
		assertEquals(Set.of(run.lineOf("@method")), lines(sites));
		assertEquals(1, sites.size(), sites.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void moduleFunctionIsNotPassedTheModule(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(DIR + "four_ways.py", config);
		assertEquals(Val.exact(1), helper.after("@module").value("a"), "echo receives 1 alone");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void callAnalysedInSeveralContextsIsListedOnce(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "contexts.py", config);
		List<Call> sites = List.copyOf(run.sitesOf(ECHO));
		assertEquals(1, sites.size(), sites.toString());
		assertEquals(Set.of(run.lineOf("@inner")), lines(sites));
	}

	/**
	 * Yields the source lines of call sites; sites the analysis synthesises
	 * with no source location, such as the initialisation of a unit, have no
	 * line.
	 */
	static Set<Integer> lines(
			Collection<Call> sites) {
		return sites.stream()
				.filter(site -> site.getLocation() instanceof SourceCodeLocation)
				.map(site -> ((SourceCodeLocation) site.getLocation()).getLine())
				.collect(Collectors.toSet());
	}
}
