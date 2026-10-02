package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import it.unive.pylisa.cfg.statement.PyInstantiation;
import it.unive.pylisa.testing.StateTestHelper;
import it.unive.pylisa.testnatives.TickNative;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that instantiating a class performs calls to {@code __new__} and to
 * {@code __init__} that the call graph resolves and records like any other
 * call: Python's order is kept ({@code __init__} is looked up after
 * {@code __new__} has run, for each way {@code __new__} may run), the created
 * object is the result, and every part that cannot be run is reported.
 */
class InstantiationTest {

	private static final String DIR = "src/test/resources/callgraph/instantiation/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void constructorsAreCallSitesHoweverTheyAreReached(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "constructors.py", config);
		Set<Integer> base = CallSitesTest.lines(run.sitesOfMethod("Base", "__init__"));
		assertTrue(base.contains(run.lineOf("@super")), "super().__init__: " + base);
		assertTrue(base.contains(run.lineOf("@explicit")), "Base.__init__(self, v): " + base);
		Set<Integer> sub = CallSitesTest.lines(run.sitesOfMethod("Sub", "__init__"));
		assertTrue(sub.containsAll(Set.of(run.lineOf("@sub"), run.lineOf("@either"))), sub.toString());
		Set<Integer> explicit = CallSitesTest.lines(run.sitesOfMethod("Explicit", "__init__"));
		assertTrue(explicit.containsAll(Set.of(run.lineOf("@explicit_call"), run.lineOf("@either"))),
				explicit.toString());
		assertEquals(Set.of(run.lineOf("@native")),
				CallSitesTest.lines(run.sitesOf("testnatives.Plain.__init__::$call")));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theCreatedObjectIsTheResult(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(DIR + "constructors.py", config);
		assertEquals(Val.exact(1), helper.after("@sub").value("s.v"));
	}

	// known to fail until a class attribute assigned outside the class body
	// (here, inside __new__) is seen by the lookup of the class's methods,
	// which needs classes modelled as objects with their attributes
	@Tag("known-failing")
	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void initReboundByNewIsTheOneCalled(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "rebinding_new.py", config);
		assertTrue(CallSitesTest.lines(run.sitesEndingWith(".other::$call")).contains(run.lineOf("@rebound")),
				run.allSites().toString());
		StateTestHelper helper = StateTestHelper.analyse(DIR + "rebinding_new.py", config);
		assertEquals(Val.exact(2), helper.after("@rebound").value("c.tag"));
	}

	// known to fail until a class attribute assigned outside the class body
	// (here, inside __new__) is seen by the lookup of the class's methods,
	// which needs classes modelled as objects with their attributes
	@Tag("known-failing")
	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void initIsLookedUpForEachWayNewMayRun(
			AnalysisConfig config)
			throws Exception {
		// one __new__ rebinds __init__ and the other does not: the inherited
		// __init__ still runs on the second
		Run run = CallGraphTestSupport.analyse(DIR + "two_news.py", config);
		int line = run.lineOf("@two");
		assertTrue(CallSitesTest.lines(run.sitesEndingWith(".other::$call")).contains(line),
				run.allSites().toString());
		assertTrue(CallSitesTest.lines(run.sitesOfMethod("Base", "__init__")).contains(line),
				run.allSites().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void constructorOfAClassWithSeveralBasesIsReported(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse("src/test/resources/programs/calls/multiple_bases.py", config);
		assertTrue(nestedOpen(run, "@multiple").stream().anyMatch(InstantiationTest::ofNew),
				run.events().toString());
		assertTrue(nestedOpen(run, "@multiple").stream().anyMatch(InstantiationTest::ofInit),
				"__init__ cannot run on the unknown object: " + run.events());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void recursionThroughNewIsAnalysed(
			AnalysisConfig config)
			throws Exception {
		// while the recursion through __new__ is solved, the inner creation
		// has no result yet; the analysis completes and goes on after the
		// outer creation
		StateTestHelper helper = StateTestHelper.analyse(DIR + "new_recursion.py", config);
		assertTrue(helper.after("@outer").isReachable());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void recursionThroughAnInstantiationIsFound(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "nested_recursion.py", config);
		assertEquals(Set.of(run.lineOf("@nested"), run.lineOf("@outer")),
				CallSitesTest.lines(run.sitesOfMethod("T", "__init__")));
		assertTrue(run.callGraph().getRecursions().stream()
				.anyMatch(cycle -> cycle.stream()
						.anyMatch(member -> member.getDescriptor().getFullName().endsWith(".__init__::$call"))),
				run.callGraph().getRecursions().toString());
		StateTestHelper helper = StateTestHelper.analyse(DIR + "nested_recursion.py", config);
		assertTrue(helper.after("@outer").value("t.c") instanceof Val.Exact,
				"the inner object is created: " + helper.after("@outer").value("t.c"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void argumentsAreEvaluatedAsManyTimesAsBefore(
			AnalysisConfig config)
			throws Exception {
		// how many times the frontend's translation evaluates an argument: a
		// call evaluates it once and applies it once; an instantiation also
		// evaluates it for __new__ and for __init__
		TickNative.APPLIED.set(0);
		StateTestHelper.analyse(DIR + "counts_call.py", config);
		assertEquals(2, TickNative.APPLIED.get());
		TickNative.APPLIED.set(0);
		StateTestHelper.analyse(DIR + "counts_instantiation.py", config);
		assertEquals(5, TickNative.APPLIED.get());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theApplicationOfAUserNewIsNotACallSite(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "user_new.py", config);
		for (Call site : run.allSites())
			assertFalse(site.getTargetName().equals("$call"), site + " at " + site.getLocation());
		assertEquals(Set.of(run.lineOf("@user")), CallSitesTest.lines(run.sitesOfMethod("U", "__new__")));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void initRunsOnTheObjectsNewReturnsAndIsOpenOnTheRest(
			AnalysisConfig config)
			throws Exception {
		// __new__ returns None on one path and an object on the other:
		// __init__ runs on the object, and is reported as a part that cannot
		// be run on None
		Run run = CallGraphTestSupport.analyse(DIR + "new_maybe_none.py", config);
		int line = run.lineOf("@maybe_none");
		assertTrue(CallSitesTest.lines(run.sitesOfMethod("C", "__init__")).contains(line),
				run.allSites().toString());
		assertTrue(nestedOpen(run, "@maybe_none").stream().anyMatch(InstantiationTest::ofInit),
				run.events().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void newWithoutAValueGivesAnUnknownObjectAndAnOpenInit(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "new_no_value.py", config);
		assertTrue(nestedOpen(run, "@no_value").stream().anyMatch(InstantiationTest::ofInit),
				run.events().toString());
		assertTrue(run.sitesOf("testnatives.Void.__init__::$call").isEmpty(), run.allSites().toString());
		StateTestHelper helper = StateTestHelper.analyse(DIR + "new_no_value.py", config);
		assertTrue(helper.after("@after").isReachable());
		assertEquals(Val.top(), helper.after("@after").value("v"));
	}

	/** Whether the call of an event is the call of {@code __init__}. */
	private static boolean ofInit(
			CallResolved event) {
		return event.getOriginal().getSubExpressions()[0].toString().endsWith("__init__");
	}

	/** Whether the call of an event is the call of {@code __new__}. */
	private static boolean ofNew(
			CallResolved event) {
		return event.getOriginal().getSubExpressions()[0].toString().endsWith("__new__");
	}

	/**
	 * Yields the open resolutions of the calls an instantiation performs on
	 * the labelled line.
	 */
	private static List<CallResolved> nestedOpen(
			Run run,
			String label) {
		return run.eventsAt(run.lineOf(label)).stream()
				.filter(event -> event.getOriginal().getParentStatement() instanceof PyInstantiation
						&& event.getResolved() instanceof OpenCall)
				.toList();
	}
}
