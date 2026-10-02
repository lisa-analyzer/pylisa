package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.matcher.AnnotationMatcher;
import it.unive.lisa.program.annotations.matcher.BasicAnnotationMatcher;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.statement.PyAssert;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A semantic check that decides, for every {@code assert} statement of the
 * analysed program, whether its condition holds in the executions that reach
 * it.
 * <p>
 * The condition is evaluated in the state right before the assertion, in each
 * analysis context of the enclosing function, and the per-context verdicts are
 * combined (see {@link AssertionVerdict#combine}). Assertions that are not
 * proved are also reported as warnings of the analysis.
 * </p>
 * <p>
 * The verdicts are as reliable as the analysis that computed the states: when
 * calls to unknown code are assumed to have no effect, a proved assertion is
 * proved only under that assumption.
 * </p>
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
public class AssertChecker<A extends AbstractLattice<A>, D extends AbstractDomain<A>> implements SemanticCheck<A, D> {

	private static final AnnotationMatcher UNSOUND = new BasicAnnotationMatcher(
			new Annotation(ParserSupport.UNSOUND_TRANSLATION));

	private final Map<CodeLocation, AssertionVerdict> verdicts = new LinkedHashMap<>();

	private final Map<CodeLocation, PyAssert> assertions = new LinkedHashMap<>();

	private boolean unsoundTranslation;

	/**
	 * Yields the verdict of every assertion checked, by the location of the
	 * assertion. The verdicts are final once the analysis has completed.
	 *
	 * @return the verdicts, in the order the assertions were checked
	 */
	public Map<CodeLocation, AssertionVerdict> getVerdicts() {
		return Collections.unmodifiableMap(verdicts);
	}

	/**
	 * Yields whether some analysed function was translated unsoundly by the
	 * frontend, in which case no verdict is definite.
	 *
	 * @return {@code true} if it was
	 */
	public boolean sawUnsoundTranslation() {
		return unsoundTranslation;
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph) {
		if (!tool.getResultOf(graph).isEmpty()
				&& graph.getDescriptor().getAnnotations().contains(UNSOUND))
			unsoundTranslation = true;
		return true;
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph,
			Statement node) {
		if (!(node instanceof PyAssert assertion))
			return true;
		Collection<AnalyzedCFG<A>> results = tool.getResultOf(graph);
		AssertionVerdict verdict = results.isEmpty() ? AssertionVerdict.NOT_ANALYSED : AssertionVerdict.UNREACHABLE;
		for (AnalyzedCFG<A> result : results)
			verdict = verdict.combine(verdictIn(tool.getAnalysis(), result, assertion));
		verdicts.merge(assertion.getLocation(), verdict, AssertionVerdict::combine);
		assertions.put(assertion.getLocation(), assertion);
		return true;
	}

	@Override
	public void afterExecution(
			SemanticTool<A, D> tool) {
		// part of the program may have been misrepresented, so that the
		// analysed executions are not all the executions: nothing is definite
		if (unsoundTranslation)
			verdicts.replaceAll((location, verdict) -> verdict.unreliable());
		verdicts.forEach((location, verdict) -> {
			PyAssert assertion = assertions.get(location);
			if (verdict == AssertionVerdict.FAILS)
				tool.warnOn(assertion, "The assertion fails in every execution that reaches it");
			else if (verdict == AssertionVerdict.MAY_FAIL)
				tool.warnOn(assertion, "The assertion may fail");
			else if (verdict == AssertionVerdict.NOT_ANALYSED)
				tool.warnOn(assertion, "The assertion was not analysed: no analysed code runs its function");
		});
	}

	private AssertionVerdict verdictIn(
			Analysis<A, D> analysis,
			AnalyzedCFG<A> result,
			PyAssert assertion) {
		try {
			AnalysisState<A> beforeAssertion = result.getAnalysisStateAfter(assertion.getCondition());
			if (beforeAssertion.getExecution().isBottom() || beforeAssertion.getExecutionState().isBottom())
				return AssertionVerdict.UNREACHABLE;
			if (beforeAssertion.getExecutionExpressions().isEmpty())
				// reached, but the condition has no value: nothing is decided
				return AssertionVerdict.MAY_FAIL;
			AssertionVerdict verdict = AssertionVerdict.UNREACHABLE;
			for (SymbolicExpression condition : beforeAssertion.getExecutionExpressions()) {
				Satisfiability satisfiability = analysis.satisfies(beforeAssertion, condition, assertion);
				verdict = verdict.combine(AssertionVerdict.of(satisfiability));
			}
			return verdict;
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot evaluate the condition of " + assertion + " at "
					+ assertion.getLocation(), e);
		}
	}
}
