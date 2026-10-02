package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.type.Type;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import it.unive.pylisa.cfg.statement.CallTargets.Python;
import it.unive.pylisa.cfg.statement.PyInstantiation;
import it.unive.pylisa.cfg.statement.PyResolvedCall;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that every call, or part of a call, that cannot be dispatched is
 * reported at that call with an open resolution, as LiSA reports the calls it
 * cannot resolve, while the analysis continues it as before.
 */
class OpenCallTest {

	private static final String DIR = "src/test/resources/callgraph/open/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeThatMayBeUnknownHasAnOpenResolution(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "unknown_on_one_path.py", config);
		List<CallResolved> events = programEvents(run, "@mixed");
		assertTrue(resolved(events).stream()
				.anyMatch(resolution -> resolution.targets().stream().anyMatch(Python.class::isInstance)),
				events.toString());
		assertOpen(events);
		StateTestHelper helper = StateTestHelper.analyse(DIR + "unknown_on_one_path.py", config);
		assertEquals(Val.top(), helper.after("@mixed").value("x"), "the unknown callee may return anything");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeThatIsPartlyDispatchableHasBothResolutions(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "non_callable.py", config);
		List<CallResolved> events = programEvents(run, "@partly");
		assertTrue(resolved(events).stream()
				.anyMatch(resolution -> resolution.targets().stream().anyMatch(Python.class::isInstance)
						&& !resolution.unresolved().isEmpty()),
				events.toString());
		assertOpen(events);
		StateTestHelper helper = StateTestHelper.analyse(DIR + "non_callable.py", config);
		assertEquals(Val.top(), helper.after("@partly").value("x"),
				"the non-callable part may raise or return anything");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void unknownCalleeHasAnOpenResolutionAndNoTarget(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "all_open.py", config);
		List<CallResolved> events = programEvents(run, "@unknown");
		assertTrue(resolved(events).stream().allMatch(resolution -> resolution.getTargets().isEmpty()),
				events.toString());
		assertOpen(events);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void decoratorThatCannotBeResolvedIsOpenAndPassesTheFunctionThrough(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "decorator.py", config);
		assertOpen(programEvents(run, "@decorator"));
		assertEquals(Set.of(run.lineOf("@call")), CallSitesTest.lines(run.sitesEndingWith(".handler::$call")),
				"the decorated function is still called");
		StateTestHelper helper = StateTestHelper.analyse(DIR + "decorator.py", config);
		assertEquals(Val.exact(1), helper.after("@call").value("h"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeWithTooManyNonCallableTypesIsWhollyOpen(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "over_limit.py", config);
		List<CallResolved> events = programEvents(run, "@over");
		assertTrue(resolved(events).stream().allMatch(resolution -> resolution.getTargets().isEmpty()),
				events.toString());
		assertOpen(events);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeBeyondTheLimitKeepsItsCallableTargetsAndIsOpen(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "beyond_limit_callable.py", config);
		List<CallResolved> events = programEvents(run, "@mix");
		assertTrue(resolved(events).stream().anyMatch(resolution -> resolution.getTargets().size() == 15),
				events.toString());
		assertOpen(events);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void openPartReachedAgainWithTheSameTypesIsReportedOnce(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "repeated.py", config);
		assertOpen(programEvents(run, "@inside"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void eachCallAndTypesIsResolvedOnce(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse("src/test/resources/callgraph/sites/contexts.py", config);
		List<CallResolved> events = programEvents(run, "@inner");
		List<PyResolvedCall> resolutions = resolved(events);
		assertFalse(resolutions.isEmpty());
		// a resolution found again posts nothing: the types of the events are
		// all different
		Set<List<Set<Type>>> types = new HashSet<>();
		for (CallResolved event : events)
			if (event.getResolved() instanceof PyResolvedCall)
				assertTrue(types.add(Arrays.asList(event.getTypes())), "posted twice: " + event);
	}

	/**
	 * Asserts that, for each array of types the call is resolved for, exactly
	 * one resolved call and exactly one open resolution are posted, the open
	 * one with the call as source and LiSA's name of the call: a
	 * resolution found again posts nothing.
	 */
	private static void assertOpen(
			List<CallResolved> events) {
		assertFalse(events.isEmpty(), "no resolution");
		Map<List<Set<Type>>, List<CallResolved>> byTypes = new HashMap<>();
		for (CallResolved event : events)
			byTypes.computeIfAbsent(Arrays.asList(event.getTypes()), types -> new ArrayList<>()).add(event);
		for (List<CallResolved> same : byTypes.values()) {
			assertEquals(1, same.stream().filter(event -> event.getResolved() instanceof PyResolvedCall).count(),
					same.toString());
			List<CallResolved> open = same.stream().filter(event -> event.getResolved() instanceof OpenCall)
					.toList();
			assertEquals(1, open.size(), "one open resolution per types: " + same);
			OpenCall call = (OpenCall) open.get(0).getResolved();
			assertEquals(open.get(0).getOriginal(), call.getSource());
			// LiSA gives a call with a source the parent of its source
			assertEquals(open.get(0).getOriginal().getParentStatement(), call.getParentStatement());
			assertEquals("__call__", call.getTargetName());
		}
	}

	/**
	 * Yields the events of the calls the program writes on the labelled line,
	 * leaving out the nested calls of any instantiation on the same line.
	 */
	private static List<CallResolved> programEvents(
			Run run,
			String label) {
		return run.eventsAt(run.lineOf(label)).stream()
				.filter(event -> !(event.getOriginal().getParentStatement() instanceof PyInstantiation))
				.toList();
	}

	/** Yields the resolved calls among the events. */
	private static List<PyResolvedCall> resolved(
			List<CallResolved> events) {
		return events.stream()
				.map(CallResolved::getResolved)
				.filter(PyResolvedCall.class::isInstance)
				.map(PyResolvedCall.class::cast)
				.toList();
	}
}
