package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.cfg.statement.CallTargets.ConstructorParts;
import it.unive.pylisa.cfg.statement.CallTargets.Instantiation;
import it.unive.pylisa.cfg.statement.CallTargets.Native;
import it.unive.pylisa.cfg.statement.CallTargets.Python;
import it.unive.pylisa.cfg.statement.CallTargets.Target;
import it.unive.pylisa.cfg.statement.CallTargets.Unresolved;
import it.unive.pylisa.cfg.statement.CallTargets;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.testing.StateTestHelper;
import it.unive.pylisa.testnatives.NoneNative;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the targets a call resolves to: every runtime type of the callee
 * either yields a target or is reported as an unresolved part, so no
 * execution of the call is silently dropped.
 */
class CallTargetsTest {

	private static final String PROGRAMS = "src/test/resources/programs/calls/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeThatMayBeUnknownHasAnUnresolvedPart(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "mixed_callee.py", config, List.of(probe));
		List<Target> targets = probe.at(helper.lineOf("@mixed"));
		assertTrue(targets.stream().anyMatch(Python.class::isInstance), targets.toString());
		assertTrue(targets.stream().anyMatch(Unresolved.class::isInstance), targets.toString());
		assertEquals(Val.top(), helper.after("@mixed").value("x"),
				"the unknown callee may return anything, not only what g returns");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void subclassWithoutInitUsesTheInheritedNativeInit(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "inherited_native_init.py", config,
				List.of(probe));
		List<Target> targets = probe.at(helper.lineOf("@sub"));
		assertTrue(targets.stream().anyMatch(Instantiation.class::isInstance), targets.toString());
		ConstructorParts parts = probe.partsAt(helper.lineOf("@sub"));
		assertTrue(parts.initialization().stream()
				.anyMatch(target -> target instanceof Native natives
						&& natives.implementation() == NoneNative.class
						&& natives.library().equals("testnatives")),
				parts.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void calleeWithTooManyTypesIsUnresolved(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "over_limit.py", config, List.of(probe));
		List<Target> targets = probe.at(helper.lineOf("@over"));
		assertTrue(targets.stream().allMatch(Unresolved.class::isInstance), targets.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void classWithSeveralBasesHasAnUnresolvedConstructor(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "multiple_bases.py", config, List.of(probe));
		ConstructorParts parts = probe.partsAt(helper.lineOf("@multiple"));
		assertTrue(parts.creation().stream().anyMatch(Unresolved.class::isInstance), parts.toString());
		assertEquals(Val.top(), helper.after("@multiple").value("m"), "the created object is unknown");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void classWhoseNewIsNotCallableHasAnUnresolvedConstructor(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "new_not_callable.py", config,
				List.of(probe));
		ConstructorParts parts = probe.partsAt(helper.lineOf("@notcallable"));
		assertTrue(parts.creation().stream().anyMatch(Unresolved.class::isInstance), parts.toString());
		assertEquals(Val.top(), helper.after("@notcallable").value("n"), "the created object is unknown");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void newReturningAValueThatIsNotAnInstanceYieldsThatValue(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "new_returns_constant.py", config);
		assertEquals(Val.exact(5), helper.after("@constant").value("k"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void plainFunctionIsAPythonTarget(
			AnalysisConfig config)
			throws Exception {
		Probe probe = new Probe();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "instantiation_in_helper.py", config,
				List.of(probe));
		List<Target> targets = probe.at(helper.lineOf("@outer")).stream().distinct().toList();
		assertEquals(1, targets.size(), targets.toString());
		assertTrue(targets.get(0) instanceof Python, targets.toString());
		assertFalse(targets.stream().anyMatch(Unresolved.class::isInstance), targets.toString());
	}

	/**
	 * Collects the targets of every call of the analysed program, by line,
	 * over every context in which the call is reached.
	 */
	private static final class Probe implements SemanticCheck<Lat, Dom> {

		private final Map<Integer, List<Target>> byLine = new HashMap<>();

		private final Map<Integer, ConstructorParts> partsByLine = new HashMap<>();

		@Override
		public boolean visit(
				SemanticTool<Lat, Dom> tool,
				CFG graph,
				Statement node) {
			for (PyCall call : calls(node))
				for (AnalyzedCFG<Lat> result : tool.getResultOf(graph))
					try {
						List<Target> targets = CallTargets.of(tool.getAnalysis(), result, call);
						int line = ((SourceCodeLocation) call.getLocation()).getLine();
						byLine.computeIfAbsent(line, l -> new ArrayList<>()).addAll(targets);
						Expression[] sub = call.getSubExpressions();
						for (Target target : targets)
							if (target instanceof Instantiation instantiation)
								partsByLine.put(line, CallTargets.constructorParts(tool.getAnalysis(),
										result.getAnalysisStateAfter(sub[sub.length - 1]), instantiation.type(),
										call));
					} catch (SemanticException e) {
						throw new IllegalStateException("Cannot resolve the targets of " + call, e);
					}
			return true;
		}

		List<Target> at(
				int line) {
			List<Target> targets = byLine.get(line);
			if (targets == null)
				throw new AssertionError("No reached call on line " + line + "; calls on lines " + byLine.keySet());
			return targets;
		}

		ConstructorParts partsAt(
				int line) {
			ConstructorParts parts = partsByLine.get(line);
			if (parts == null)
				throw new AssertionError("No instantiation on line " + line + "; on lines " + partsByLine.keySet());
			return parts;
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
