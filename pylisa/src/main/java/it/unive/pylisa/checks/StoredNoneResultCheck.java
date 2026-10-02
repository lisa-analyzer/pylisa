package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.matcher.AnnotationMatcher;
import it.unive.lisa.program.annotations.matcher.BasicAnnotationMatcher;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.statement.CallTargets;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Finds assignments that store the result of a call that always returns
 * {@code None}, such as {@code t = Thread(...).start()}: in every analysed
 * context that reaches the assignment, every runtime type of the result is
 * {@code NoneType}. A result with no value (no execution returns) is not
 * {@code None}, and a call of a Python function translated unsoundly (a
 * generator, an {@code async def}) is never reported, since the frontend may
 * make it return {@code None} where Python returns an object.
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
public class StoredNoneResultCheck<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		SemanticCheck<A, D> {

	private static final AnnotationMatcher UNSOUND = new BasicAnnotationMatcher(
			new Annotation(ParserSupport.UNSOUND_TRANSLATION));

	private static final AnnotationMatcher LIMITATION = new BasicAnnotationMatcher(
			new Annotation(ParserSupport.KNOWN_LIMITATION));

	private final List<Advisory> advisories = new ArrayList<>();

	private boolean unsoundTranslation;

	/**
	 * Yields the advisories of the last run.
	 *
	 * @return the advisories, in the order the statements were visited
	 */
	public List<Advisory> advisories() {
		return List.copyOf(advisories);
	}

	@Override
	public void beforeExecution(
			SemanticTool<A, D> tool) {
		advisories.clear();
		unsoundTranslation = false;
	}

	/**
	 * Yields whether some analysed function was translated unsoundly, or with
	 * a known limitation, by the frontend. The states of such a function may miss
	 * executions, and so may the states of every function it returns to, so
	 * that no advisory of the run is then definite.
	 *
	 * @return {@code true} if one was
	 */
	public boolean sawUnsoundTranslation() {
		return unsoundTranslation;
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph) {
		// a known limitation of the frontend may also make a value look
		// certain where Python gives another
		if (!tool.getResultOf(graph).isEmpty() && (graph.getDescriptor().getAnnotations().contains(UNSOUND)
				|| graph.getDescriptor().getAnnotations().contains(LIMITATION)))
			unsoundTranslation = true;
		return true;
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph,
			Statement node) {
		if (!(node instanceof Assignment assignment) || !(assignment.getRight() instanceof PyCall call))
			return true;
		boolean reached = false;
		for (AnalyzedCFG<A> result : tool.getResultOf(graph)) {
			AnalysisState<A> after = result.getAnalysisStateAfter(call);
			if (after.getExecution().isBottom() || after.getExecutionState().isBottom())
				continue;
			if (!alwaysNone(tool, result, after, call))
				return true;
			reached = true;
		}
		if (reached)
			advisories.add(new Advisory(Advisory.Kind.STORED_NONE_RESULT, node.getLocation(),
					"the result of " + call + " is always None, and it is stored"));
		return true;
	}

	private boolean alwaysNone(
			SemanticTool<A, D> tool,
			AnalyzedCFG<A> result,
			AnalysisState<A> after,
			PyCall call) {
		try {
			for (CallTargets.Target target : CallTargets.of(tool.getAnalysis(), result, call))
				if (target instanceof CallTargets.Python python
						&& python.cfg().getDescriptor().getAnnotations().contains(UNSOUND))
					return false;
			if (after.getExecutionExpressions().isEmpty())
				return false;
			for (SymbolicExpression value : after.getExecutionExpressions()) {
				Set<Type> types = tool.getAnalysis().getRuntimeTypesOf(after, value, call);
				if (types.isEmpty() || !types.stream().allMatch(Type::isNullType))
					return false;
			}
			return true;
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot type the result of " + call + " at " + call.getLocation(), e);
		}
	}
}
