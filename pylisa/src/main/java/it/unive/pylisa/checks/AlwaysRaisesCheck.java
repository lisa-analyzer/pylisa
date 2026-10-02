package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.interprocedural.callgraph.CallGraphNode;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.matcher.AnnotationMatcher;
import it.unive.lisa.program.annotations.matcher.BasicAnnotationMatcher;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.statement.CallTargets.Instantiation;
import it.unive.pylisa.cfg.statement.CallTargets.Native;
import it.unive.pylisa.cfg.statement.CallTargets.Target;
import it.unive.pylisa.cfg.statement.CallTargets;
import it.unive.pylisa.cfg.statement.PyAssert;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.cfg.statement.PyRaise;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.libraries.natives.LibraryNative;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * A semantic check that finds the statements performing a call that no
 * execution completes normally: in every analysis context where some
 * execution reaches the statement, every such execution that completes ends
 * with an exception. Each finding names the exceptions the statement adds and
 * the statements that raise them.
 * <p>
 * An execution may also not complete at all, when a called function loops
 * forever. A statement that calls a library function which may block, such as
 * an event loop that returns only when stopped, is never reported, since its
 * missing normal result is not a failure (see {@link LibraryNative#diverges()}).
 * </p>
 * <p>
 * The verdict is as reliable as the analysis: a function translated unsoundly
 * by the frontend may make a call look as if it always raised where Python
 * does not, so that every finding is then marked as unreliable.
 * </p>
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
public class AlwaysRaisesCheck<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		SemanticCheck<A, D> {

	private static final AnnotationMatcher UNSOUND = new BasicAnnotationMatcher(
			new Annotation(ParserSupport.UNSOUND_TRANSLATION));

	private static final AnnotationMatcher LIMITATION = new BasicAnnotationMatcher(
			new Annotation(ParserSupport.KNOWN_LIMITATION));

	/**
	 * A statement that always raises.
	 *
	 * @param statement  the statement
	 * @param exceptions the names of the exceptions it adds, sorted
	 * @param raisers    the locations of the statements of the same function
	 *                       raising them, sorted; an exception raised inside a
	 *                       called function is attributed to the call
	 */
	public record Finding(Statement statement, Set<String> exceptions, Set<String> raisers) {
	}

	private final List<Finding> findings = new ArrayList<>();

	private boolean unsoundTranslation;

	/**
	 * Yields the statements found, in the order they were checked. They are
	 * final once the analysis has completed.
	 *
	 * @return the findings
	 */
	public List<Finding> getFindings() {
		return Collections.unmodifiableList(findings);
	}

	/**
	 * Yields whether some analysed function was translated unsoundly by the
	 * frontend, in which case no finding is definite.
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
		// a known limitation of the frontend may also make a call look as if
		// it always raised
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
		// raising is what a raise or a failing assert is for; a call inside a
		// statement is judged with its statement
		if (!graph.containsNode(node) || node instanceof PyRaise || node instanceof PyAssert || !performsCall(node))
			return true;
		Collection<AnalyzedCFG<A>> results = tool.getResultOf(graph);
		boolean reached = false;
		Set<String> exceptions = new TreeSet<>();
		Set<String> raisers = new TreeSet<>();
		try {
			for (AnalyzedCFG<A> result : results) {
				AnalysisState<A> before = result.getAnalysisStateBefore(node);
				if (unreachable(before))
					continue;
				AnalysisState<A> after = result.getAnalysisStateAfter(node);
				if (!unreachable(after))
					// some execution completes the statement in this context
					return true;
				reached = true;
				Set<AnalysisState.Error> previous = keys(before.getErrors().getKeys());
				for (AnalysisState.Error error : keys(after.getErrors().getKeys()))
					if (!previous.contains(error)) {
						exceptions.add(error.getType().toString());
						// an error raised inside a called function is
						// attributed to the call
						if (error.getThrower() != node && !node.getLocation().equals(error.getThrower().getLocation()))
							raisers.add(error.getThrower().getLocation().toString());
					}
				Set<Type> smashed = keys(before.getSmashedErrors().getKeys());
				for (Type type : keys(after.getSmashedErrors().getKeys()))
					if (!smashed.contains(type))
						exceptions.add(type.toString());
			}
		} catch (SemanticException e) {
			throw new IllegalStateException("Cannot read the states around " + node + " at " + node.getLocation(),
					e);
		}
		if (reached && !exceptions.isEmpty() && !mayBlock(tool.getAnalysis(), results, node))
			findings.add(new Finding(node, exceptions, raisers));
		return true;
	}

	@Override
	public void afterExecution(
			SemanticTool<A, D> tool) {
		Set<CodeMember> unseen = mayRunUnseen(tool);
		findings.removeIf(finding -> unseen.contains(finding.statement().getCFG()));
		String reliability = unsoundTranslation
				? " (best-effort: part of the program was translated unsoundly)"
				: "";
		for (Finding finding : findings)
			tool.warnOn(finding.statement(),
					"Every execution of this statement that completes raises an exception, such as "
							+ finding.exceptions()
							+ (finding.raisers().isEmpty() ? "" : ", raised at " + finding.raisers()) + reliability);
	}

	/**
	 * Yields whether a statement may call, in some context, a library
	 * function that may block instead of returning.
	 */
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean mayBlock(
			Analysis<A, D> analysis,
			Collection<AnalyzedCFG<A>> results,
			Statement statement) {
		if (statement instanceof LibraryNative library && library.diverges())
			return true;
		if (statement instanceof PyCall call)
			for (AnalyzedCFG<A> result : results)
				try {
					AnalysisState<A> applied = result.getAnalysisStateAfter(
							call.getSubExpressions()[call.getSubExpressions().length - 1]);
					for (Target target : CallTargets.of(analysis, result, call))
						if (mayBlock(analysis, applied, call, target))
							return true;
				} catch (SemanticException e) {
					throw new IllegalStateException(
							"Cannot resolve the targets of " + call + " at " + call.getLocation(), e);
				}
		for (Expression sub : subExpressions(statement))
			if (mayBlock(analysis, results, sub))
				return true;
		return false;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean mayBlock(
			Analysis<A, D> analysis,
			AnalysisState<A> applied,
			PyCall call,
			Target target)
			throws SemanticException {
		return switch (target) {
		case Native natives -> LibraryNative.diverges(natives.implementation());
		case Instantiation instantiation -> {
			CallTargets.ConstructorParts parts = CallTargets.constructorParts(analysis, applied,
					instantiation.type(), call);
			boolean blocks = false;
			for (Target part : parts.creation())
				blocks |= mayBlock(analysis, applied, call, part);
			for (Target part : parts.initialization())
				blocks |= mayBlock(analysis, applied, call, part);
			yield blocks;
		}
		default -> false;
		};
	}

	private static Expression[] subExpressions(
			Statement statement) {
		return statement instanceof NaryExpression expression ? expression.getSubExpressions()
				: statement instanceof NaryStatement nary ? nary.getSubExpressions() : new Expression[0];
	}

	/**
	 * Yields the functions that may run in contexts the analysis does not
	 * see: those whose function or class is used as a value somewhere, other
	 * than as the callee of a call (a callback handed to a library, a
	 * converter, a function stored to be called later), and every function they
	 * call. The whole program is scanned, also the functions the analysis never
	 * reached; a function that escapes makes every name it mentions escape too,
	 * until no new function matches, since the calls it makes in contexts the
	 * analysis does not see are in no call graph. A function is matched by its
	 * simple name or by the simple name of its class, which over-approximates
	 * the functions used as values.
	 */
	private static Set<CodeMember> mayRunUnseen(
			SemanticTool<?, ?> tool) {
		// every function of the program, also those the analysis never
		// reached: a function nobody calls may still hand another one on
		Set<CFG> cfgs = new HashSet<>();
		Set<CodeMember> reached = new HashSet<>();
		for (CallGraphNode node : tool.getCallGraph().getNodes()) {
			reached.add(node.getCodeMember());
			if (node.getCodeMember() instanceof CFG cfg)
				cfgs.addAll(cfg.getDescriptor().getUnit().getProgram().getAllCFGs());
		}
		Set<String> values = new HashSet<>();
		for (CFG cfg : cfgs)
			for (Statement statement : cfg.getNodes())
				namesUsedAsValues(statement, false, values);
		// a function the analysis never reached, run by code it does not see,
		// runs whatever it calls unseen too: its names spread until no new
		// function matches
		Set<CodeMember> escaping = new HashSet<>();
		boolean grown = true;
		while (grown) {
			grown = false;
			for (CFG cfg : cfgs)
				if (!escaping.contains(cfg) && matches(cfg, values)) {
					escaping.add(cfg);
					grown = true;
					// the call graph has only the calls of the contexts the
					// analysis saw: names also cover the others
					for (Statement statement : cfg.getNodes())
						namesIn(statement, values);
				}
		}
		Set<CodeMember> unseen = new HashSet<>(escaping);
		Set<CodeMember> analysed = new HashSet<>(escaping);
		analysed.retainAll(reached);
		if (!analysed.isEmpty())
			unseen.addAll(tool.getCallGraph().getCalleesTransitively(analysed));
		return unseen;
	}

	/**
	 * Yields whether a function is named by one of the given names: its own
	 * simple name, or the simple name of its class.
	 */
	private static boolean matches(
			CFG cfg,
			Set<String> names) {
		String[] unit = cfg.getDescriptor().getUnit().getName().split("\\.");
		return names.contains(cfg.getDescriptor().getName()) || names.contains(unit[unit.length - 1])
				|| unit.length > 1 && names.contains(unit[unit.length - 2]);
	}

	/**
	 * Collects the names an expression reads as values: names and attributes,
	 * except the name or attribute a call is applied to.
	 */
	private static void namesUsedAsValues(
			Statement statement,
			boolean callee,
			Set<String> names) {
		if (statement instanceof PyNameRef name && !callee)
			names.add(simple(name.getName()));
		if (statement instanceof AttributeAccess attribute && !callee)
			names.add(attribute.getTarget());
		if (statement instanceof LambdaExpression lambda) {
			// the lambda is a value, and whatever its body calls runs when
			// the lambda does
			namesIn(lambda.getBody(), names);
			return;
		}
		Expression[] sub = subExpressions(statement);
		for (int i = 0; i < sub.length; i++)
			// the callee of a call is its first sub-expression; the receiver of
			// a called attribute is still read as a value
			namesUsedAsValues(sub[i], statement instanceof PyCall && i == 0, names);
	}

	/**
	 * Collects every name and attribute an expression mentions, called or
	 * not.
	 */
	private static void namesIn(
			Statement statement,
			Set<String> names) {
		if (statement instanceof PyNameRef name)
			names.add(simple(name.getName()));
		if (statement instanceof AttributeAccess attribute)
			names.add(attribute.getTarget());
		if (statement instanceof LambdaExpression lambda)
			namesIn(lambda.getBody(), names);
		for (Expression sub : subExpressions(statement))
			namesIn(sub, names);
	}

	private static String simple(
			String name) {
		int separator = Math.max(name.lastIndexOf('.'), name.lastIndexOf(':'));
		return name.substring(separator + 1);
	}

	private static boolean unreachable(
			AnalysisState<?> state) {
		return state.getExecution().isBottom() || state.getExecutionState().isBottom();
	}

	private static <T> Set<T> keys(
			Set<T> keys) {
		return keys == null ? Set.of() : new HashSet<>(keys);
	}

	/**
	 * Yields whether a statement performs a call, directly or in one of its
	 * sub-expressions.
	 *
	 * @param statement the statement
	 *
	 * @return {@code true} if it does
	 */
	private static boolean performsCall(
			Statement statement) {
		if (statement instanceof Call || statement instanceof LibraryNative)
			return true;
		for (Expression expression : subExpressions(statement))
			if (performsCall(expression))
				return true;
		return false;
	}
}
