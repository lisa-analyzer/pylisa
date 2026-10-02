package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.cfg.statement.CallTargets;
import it.unive.pylisa.cfg.statement.CallTargets.Python;
import it.unive.pylisa.cfg.statement.CallTargets.Target;
import it.unive.pylisa.cfg.statement.CallTargets.Unresolved;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.type.NoInfoType;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the dispatch rule that turns the runtime types of a callee into the
 * targets of a call. The targets depend on the types alone, so the types must
 * already say which parts of the callee cannot be dispatched, and the limit on
 * the number of types applies to each callee expression on its own.
 */
class CallTargetsRuleTest {

	private static final String PROGRAM = "src/test/resources/callgraph/rule/limits_and_receivers.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void twoCalleeExpressionsWithinTheLimitAreDispatchedTogether(
			AnalysisConfig config)
			throws Exception {
		Probe probe = analyse(config);
		Seen first = probe.at(probe.helper.lineOf("@a"));
		Seen second = probe.at(probe.helper.lineOf("@b"));
		// a callee with two expressions is built from the callee values of
		// two calls, read where the second call is applied, where both
		// variables still hold them: 15 and 10 functions, more than the limit
		// together but within it each
		ExpressionSet callees = first.callees.lub(second.callees);
		Set<Type> types = CallTargets.calleeTypes(second.analysis, second.applied, callees, second.call);
		assertEquals(25, types.stream().filter(PyFunctionType.class::isInstance).count(), types.toString());
		assertFalse(types.contains(NoInfoType.INSTANCE), types.toString());
		List<Target> targets = CallTargets.targets(types);
		assertEquals(25, targets.stream().filter(Python.class::isInstance).count(), targets.toString());
		assertFalse(targets.stream().anyMatch(Unresolved.class::isInstance), targets.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeWithTooManyCallableTypesIsWhollyOpen(
			AnalysisConfig config)
			throws Exception {
		Probe probe = analyse(config);
		Set<Type> types = probe.calleeTypes("@all");
		assertEquals(Set.of(NoInfoType.INSTANCE), types);
		List<Target> targets = CallTargets.targets(types);
		assertEquals(1, targets.size(), targets.toString());
		assertTrue(targets.get(0) instanceof Unresolved, targets.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeBeyondTheLimitKeepsItsCallableTypes(
			AnalysisConfig config)
			throws Exception {
		Probe probe = analyse(config);
		Set<Type> types = probe.calleeTypes("@mix");
		assertEquals(15, types.stream().filter(PyFunctionType.class::isInstance).count(), types.toString());
		assertTrue(types.contains(NoInfoType.INSTANCE), "the types beyond the limit are an open part: " + types);
		assertEquals(16, types.size(), types.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeWithTooManyTypesNoneCallableIsWhollyOpen(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		probe.helper = StateTestHelper.analyse("src/test/resources/programs/calls/over_limit.py", config,
				List.of(probe));
		assertEquals(Set.of(NoInfoType.INSTANCE), probe.calleeTypes("@over"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeThatMayBeUnknownCarriesTheMarker(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		probe.helper = StateTestHelper.analyse("src/test/resources/programs/calls/mixed_callee.py", config,
				List.of(probe));
		Set<Type> types = probe.calleeTypes("@mixed");
		assertTrue(types.contains(NoInfoType.INSTANCE), types.toString());
		assertTrue(types.stream().anyMatch(PyFunctionType.class::isInstance), types.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeWithNoValueHasNoTypes(
			AnalysisConfig config)
			throws Exception {
		Seen seen = analyse(config).at("@a");
		assertEquals(Set.of(), CallTargets.calleeTypes(seen.analysis, seen.applied, new ExpressionSet(), seen.call));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void unknownCalleeIsTheMarkerAlone(
			AnalysisConfig config)
			throws Exception {
		Seen seen = analyse(config).at("@a");
		ExpressionSet unknown = new ExpressionSet(new PushAny(Untyped.INSTANCE, seen.call.getLocation()));
		assertEquals(Set.of(NoInfoType.INSTANCE),
				CallTargets.calleeTypes(seen.analysis, seen.applied, unknown, seen.call));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void libraryGlobalTheStateDoesNotTypeGetsItsRegisteredTypeAndTheMarker(
			AnalysisConfig config)
			throws Exception {
		Seen seen = analyse(config).at("@a");
		// a library function referred to by its qualified name, which the
		// state does not carry
		ExpressionSet global = new ExpressionSet(
				new GlobalVariable(Untyped.INSTANCE, "testnatives.echo", seen.call.getLocation()));
		Set<Type> types = CallTargets.calleeTypes(seen.analysis, seen.applied, global, seen.call);
		assertTrue(types.contains(PyFunctionType.lookup("testnatives.echo")), types.toString());
		assertTrue(types.contains(NoInfoType.INSTANCE), "the type is not derived by the analysis: " + types);
		assertEquals(2, types.size(), types.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void receiverThatMayBeAModuleOrUntypedIsPassed(
			AnalysisConfig config)
			throws Exception {
		Seen seen = analyse(config).at("@module");
		ExpressionSet receivers = seen.receivers
				.lub(new ExpressionSet(new GlobalVariable(Untyped.INSTANCE, "no_such_global", seen.call.getLocation())));
		Set<Type> types = CallTargets.receiverTypes(seen.analysis, seen.applied, receivers, seen.call);
		assertTrue(types.contains(NoInfoType.INSTANCE), types.toString());
		assertTrue(types.stream().anyMatch(PyModuleType.class::isInstance), types.toString());
		assertTrue(CallTargets.receiverPassed(types));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void receiverIsPassedUnlessEveryValueIsAModule(
			AnalysisConfig config)
			throws Exception {
		Probe probe = analyse(config);
		assertTrue(CallTargets.receiverPassed(probe.receiverTypes("@object")));
		assertFalse(CallTargets.receiverPassed(probe.receiverTypes("@module")));
	}

	@Test
	void receiverWithNoValueOrAnUntypedValueIsPassed() {
		assertTrue(CallTargets.receiverPassed(Set.of()));
		assertTrue(CallTargets.receiverPassed(Set.of(NoInfoType.INSTANCE)));
	}

	@Test
	void calleeWithNoValueIsAnOpenPart() {
		List<Target> targets = CallTargets.targets(Set.of());
		assertEquals(1, targets.size(), targets.toString());
		assertTrue(targets.get(0) instanceof Unresolved, targets.toString());
	}

	private static Probe analyse(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		probe.helper = StateTestHelper.analyse(PROGRAM, config, List.of(probe));
		return probe;
	}

	/**
	 * What the probe saw of a call: the values of its callee, the state the
	 * call is applied to, and the values of its receiver.
	 */
	private record Seen(
			Analysis<Lat, Dom> analysis,
			PyCall call,
			ExpressionSet callees,
			ExpressionSet receivers,
			AnalysisState<Lat> applied) {
	}

	/**
	 * Records, for every call of the analysed program, its callee and receiver
	 * values and the state it is applied to, by line.
	 */
	private static final class Probe implements SemanticCheck<Lat, Dom> {

		private final Map<Integer, Seen> byLine = new HashMap<>();

		private StateTestHelper helper;

		@Override
		public boolean visit(
				SemanticTool<Lat, Dom> tool,
				CFG graph,
				Statement node) {
			for (PyCall call : calls(node))
				for (AnalyzedCFG<Lat> result : tool.getResultOf(graph)) {
					Expression[] sub = call.getSubExpressions();
					ExpressionSet receivers = sub.length < 2 ? new ExpressionSet()
							: result.getAnalysisStateAfter(sub[1]).getExecutionExpressions();
					byLine.put(((SourceCodeLocation) call.getLocation()).getLine(),
							new Seen(tool.getAnalysis(), call,
									result.getAnalysisStateAfter(sub[0]).getExecutionExpressions(),
									receivers, result.getAnalysisStateAfter(sub[sub.length - 1])));
				}
			return true;
		}

		Seen at(
				String label) {
			return at(helper.lineOf(label));
		}

		Seen at(
				int line) {
			Seen seen = byLine.get(line);
			if (seen == null)
				throw new AssertionError("No reached call on line " + line + "; calls on lines " + byLine.keySet());
			return seen;
		}

		Set<Type> calleeTypes(
				String label)
				throws SemanticException {
			Seen seen = at(helper.lineOf(label));
			return CallTargets.calleeTypes(seen.analysis, seen.applied, seen.callees, seen.call);
		}

		Set<Type> receiverTypes(
				String label)
				throws SemanticException {
			Seen seen = at(helper.lineOf(label));
			return CallTargets.receiverTypes(seen.analysis, seen.applied, seen.receivers, seen.call);
		}

		private static List<PyCall> calls(
				Statement statement) {
			List<PyCall> found = new ArrayList<>();
			if (statement instanceof PyCall call)
				found.add(call);
			if (statement instanceof NaryExpression expression)
				for (Expression sub : expression.getSubExpressions())
					found.addAll(calls(sub));
			return found;
		}
	}

	/** The lattice of the probe: any, since the probe only forwards it. */
	private interface Lat extends AbstractLattice<Lat> {
	}

	/** The domain of the probe: any, since the probe only forwards it. */
	private interface Dom extends AbstractDomain<Lat> {
	}
}
