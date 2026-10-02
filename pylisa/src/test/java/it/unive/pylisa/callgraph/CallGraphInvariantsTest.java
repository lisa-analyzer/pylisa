package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.PyCallGraph;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import it.unive.pylisa.cfg.statement.CallTargets.Instantiation;
import it.unive.pylisa.cfg.statement.CallTargets.Python;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.statement.PyInstantiation;
import it.unive.pylisa.cfg.statement.PyResolvedCall;
import it.unive.pylisa.testing.ErrorSite;
import it.unive.pylisa.testing.Point;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the properties of pylisa's call graph that every consumer relies
 * on: one resolution per call and types, synthetic calls anchored in the
 * program, no internal application among the call sites, no state carried
 * from one run to the next, recursion found through variables, and errors
 * raised by the callable that raises them.
 */
class CallGraphInvariantsTest {

	private static final String DIR = "src/test/resources/callgraph/invariants/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeThatIsAFunctionOrAClassHasOneResolutionWithBoth(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "function_or_class.py", config);
		// the nested calls of the instantiation are on the same line
		List<PyResolvedCall> resolutions = run.eventsAt(run.lineOf("@either")).stream()
				.filter(event -> !(event.getOriginal().getParentStatement() instanceof PyInstantiation))
				.map(CallResolved::getResolved)
				.filter(PyResolvedCall.class::isInstance)
				.map(PyResolvedCall.class::cast)
				.toList();
		assertEquals(1, resolutions.size(), resolutions.toString());
		assertTrue(resolutions.get(0).targets().stream().anyMatch(Python.class::isInstance),
				resolutions.get(0).targets().toString());
		assertTrue(resolutions.get(0).targets().stream().anyMatch(Instantiation.class::isInstance),
				resolutions.get(0).targets().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void recursionThroughAVariableIsARecursionOfTheCallGraph(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "recursion.py", config);
		assertTrue(run.callGraph().getRecursions().stream()
				.anyMatch(cycle -> cycle.stream().anyMatch(member -> name(member).endsWith(".count::$call"))),
				run.callGraph().getRecursions().toString());
		StateTestHelper helper = StateTestHelper.analyse(DIR + "recursion.py", config);
		// the concrete result is 3; the analysis gives up on the recursion
		assertEquals(Val.top(), helper.after("@direct").value("x"));
		assertEquals(Val.top(), helper.after("@variable").value("y"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void syntheticCallsAreSitesThatChainToAStatementOfTheirGraph(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "synthetic.py", config);
		// the super() call, the nested __init__ of an instantiation and a
		// decorator application are calls the frontend or the analysis builds
		assertEquals(Set.of(run.lineOf("@super")),
				CallSitesTest.lines(run.sitesOf("testnatives.Plain.__init__::$call")));
		assertTrue(CallSitesTest.lines(run.sitesEndingWith(".__init__::$call")).contains(run.lineOf("@child")),
				run.allSites().toString());
		assertTrue(CallSitesTest.lines(run.sitesEndingWith(".keep::$call")).contains(run.lineOf("@decorator")),
				run.allSites().toString());
		for (Call site : run.allSites())
			if (site instanceof PyCall)
				assertTrue(anchored(site), site + " at " + site.getLocation() + " reaches no node of its graph");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void noInternalApplicationIsACallSite(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "synthetic.py", config);
		for (Call site : run.allSites())
			assertFalse(site.getTargetName().equals("$call"), site + " at " + site.getLocation());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aSecondRunCarriesNothingFromTheFirst(
			AnalysisConfig config)
			throws Exception {
		// a resolution left over from the first run would be found again,
		// and the second run would post no event and record no site for it
		PyCallGraph callGraph = new PyCallGraph();
		Run first = CallGraphTestSupport.analyse(DIR + "recursion.py", config, callGraph);
		Run second = CallGraphTestSupport.analyse(DIR + "recursion.py", config, callGraph);
		assertFalse(first.events().isEmpty());
		assertEquals(first.events().size(), second.events().size(), second.events().toString());
		assertEquals(CallSitesTest.lines(first.allSites()), CallSitesTest.lines(second.allSites()));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void receiverThatIsAModuleOnSomePathsIsPassed(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(DIR + "module_or_object.py", config);
		List<PyResolvedCall> resolutions = run.eventsAt(run.lineOf("@either")).stream()
				.map(CallResolved::getResolved)
				.filter(PyResolvedCall.class::isInstance)
				.map(PyResolvedCall.class::cast)
				.toList();
		assertFalse(resolutions.isEmpty());
		for (PyResolvedCall resolution : resolutions)
			assertEquals(2, resolution.applications().size(), "the native echo and Holder.echo: " + resolution);
		// the call passes the receiver and 1 to every target, the native echo
		// of the module included, unless every value of the receiver is a
		// module
		for (PyResolvedCall resolution : resolutions)
			for (Call application : resolution.applications())
				assertEquals(2, application.getParameters().length, application.toString());
	}

	/**
	 * Pins how calls are bound today. pylisa passes the receiver of
	 * {@code e.a(...)} whenever {@code e} is not a module, and a function read
	 * from an attribute is the plain function: a method called through a
	 * variable gets its first argument as {@code self}, a method called through
	 * its class gets the class, and a static method gets the receiver. Python
	 * binds these calls differently; this is a known limitation of the
	 * frontend, and these expectations change when binding is modelled.
	 */
	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void callsAreBoundAsToday(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(DIR + "binding.py", config);
		assertEquals(Val.exact(7), helper.after("@through").value("a"), "Python gives 1");
		assertEquals(Val.top(), helper.after("@unbound").value("b"), "Python gives 2");
		assertEquals(Val.top(), helper.after("@static").value("d"), "Python gives 3");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void errorOfALibraryCallIsRaisedByTheCallable(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(DIR + "raising.py", config);
		Point after = helper.after("@fail");
		assertTrue(after.errorSites()
				.contains(new ErrorSite("builtins.ValueError", "testnatives.may_fail", helper.lineOf("@fail"))),
				after.errorSites().toString());
		assertTrue(after.everyErrorRaisedWithinProgram(), after.errorSites().toString());
	}

	private static boolean anchored(
			Statement statement) {
		Statement current = statement;
		while (current != null) {
			Statement candidate = current;
			// a synthetic statement may equal a node: only the node itself counts
			if (current.getCFG().getNodes().stream().anyMatch(node -> node == candidate))
				return true;
			current = current instanceof Expression expression ? expression.getParentStatement() : null;
		}
		return false;
	}

	private static String name(
			CodeMember member) {
		return member.getDescriptor().getFullName();
	}
}
