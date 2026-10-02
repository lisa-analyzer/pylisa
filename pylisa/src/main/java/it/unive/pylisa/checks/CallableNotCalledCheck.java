package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.matcher.AnnotationMatcher;
import it.unive.lisa.program.annotations.matcher.BasicAnnotationMatcher;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.FalseEdge;
import it.unive.lisa.program.cfg.edge.TrueEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.ImportClass;
import it.unive.pylisa.cfg.statement.ImportFunction;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Finds statements that only name a callable (a function, a method, a class)
 * without calling it, such as {@code shutdown} written for
 * {@code shutdown()}: a statement whose value is callable in every analysed
 * context that reaches it. Assignments, calls, imports and the conditions of
 * branches use their value, so they are not such statements.
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
public class CallableNotCalledCheck<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		SemanticCheck<A, D> {

	private static final String MAIN = "__main__.";

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
		if (!isExpressionStatement(graph, node))
			return true;
		Set<Type> types = new HashSet<>();
		for (AnalyzedCFG<A> result : tool.getResultOf(graph)) {
			AnalysisState<A> after = result.getAnalysisStateAfter(node);
			if (after.getExecution().isBottom() || after.getExecutionState().isBottom())
				continue;
			Set<Type> here = typesOf(tool, after, node);
			if (here.isEmpty())
				return true;
			types.addAll(here);
		}
		if (!types.isEmpty() && types.stream().allMatch(CallableNotCalledCheck::callable)) {
			String kind = types.stream().allMatch(PyClassType.class::isInstance) ? "a class" : "a function";
			advisories.add(new Advisory(Advisory.Kind.CALLABLE_NOT_CALLED, node.getLocation(),
					sourceName(node) + " is " + kind + " and is not called"));
		}
		return true;
	}

	/**
	 * Yields whether a node of a CFG is an expression written as a statement
	 * of its own, whose value is not used.
	 */
	private static boolean isExpressionStatement(
			CFG graph,
			Statement node) {
		// checks are also given the sub-expressions of statements
		return graph.containsNode(node) && node instanceof Expression && !(node instanceof Assignment)
				&& !(node instanceof Call) && !isImport(node)
				&& !(node instanceof PyCall) && node.getLocation() instanceof SourceCodeLocation
				&& graph.getOutgoingEdges(node).stream()
						.noneMatch(edge -> edge instanceof TrueEdge || edge instanceof FalseEdge);
	}

	/**
	 * Yields the name of an expression as the program writes it: pylisa
	 * qualifies names with {@code ::} and with the module {@code __main__}.
	 */
	private static String sourceName(
			Statement node) {
		String name = node.toString().replace("::", ".");
		return name.startsWith(MAIN) ? name.substring(MAIN.length()) : name;
	}

	/**
	 * Yields whether a node is an import, whose value is what it imports.
	 */
	private static boolean isImport(
			Statement node) {
		return node instanceof ImportModule || node instanceof FromImport || node instanceof ImportClass
				|| node instanceof ImportFunction;
	}

	private Set<Type> typesOf(
			SemanticTool<A, D> tool,
			AnalysisState<A> state,
			Statement node) {
		Set<Type> types = new HashSet<>();
		try {
			for (SymbolicExpression value : state.getExecutionExpressions())
				types.addAll(tool.getAnalysis().getRuntimeTypesOf(state, value, node));
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot type " + node + " at " + node.getLocation(), e);
		}
		return types;
	}

	private static boolean callable(
			Type type) {
		return type instanceof PyFunctionType || type instanceof PyLambdaType || type instanceof PyClassType;
	}
}
