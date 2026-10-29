package it.unive.pylisa.libraries.functools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyValueCall;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * Native implementation of {@code functools.reduce(function, iterable[,
 * initial])}, that calls {@code function(acc, x)} for each element {@code x} of
 * {@code iterable}, where {@code acc} is the result of the previous call,
 * starting from {@code initial} (or from the first element, if it is not
 * given). As the number of elements is not tracked, the result is any of the
 * values {@code acc} might have: {@code initial} (or any element), and the
 * results of calling {@code function} on any of them and any element (see
 * {@link PyValueCall#call}), until they are stable (widening them after
 * {@value #WIDEN_AFTER} calls); {@code initial} (or the first element) is not a
 * result if {@code function} is surely called, that is, if the iterable is a
 * list that surely has an element (two, if {@code initial} is not given). If
 * {@code iterable} is a list of length 1 and {@code initial} is not given, its
 * element is the result, and {@code function} is not called. {@code TypeError}
 * is raised if {@code iterable} might not be iterable (see
 * {@link Iteration#iterable}), or if it might be empty and {@code initial} is
 * not given: surely, if it is a list of length 0; otherwise, unless it is a
 * list that surely has an element (see {@link ListModel#NONEMPTY}).
 */
public class Reduce extends PyNative {

	// the number of calls after which the values of acc are widened
	private static final int WIDEN_AFTER = 3;

	protected Reduce(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "reduce", params);
	}

	public static Reduce build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Reduce(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Expression[] subs = getSubExpressions();
		Expression iterableExpr = subs.length > 1 ? subs[1] : null;
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression iterable : params[1]) {
			Satisfiability it = Iteration.iterable(analysis, context, state, iterable, st);
			if (it != Satisfiability.SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
			if (it == Satisfiability.NOT_SATISFIED)
				continue;

			Set<Type> types = analysis.getRuntimeTypesOf(state, iterable, context, st);
			boolean list = !types.isEmpty() && types.stream().allMatch(ListModel::isList);
			Integer length = list ? ListModel.length(analysis, context, state, iterable, st) : null;
			// the number of elements that the iterable surely has
			int surely = length != null ? length
					: !list ? 0
							: Math.max(ListModel.shape(analysis, context, state, iterable, st).prefix(),
									ListModel.nonEmpty(analysis, context, state, iterable, st) ? 1 : 0);
			for (SymbolicExpression initial : params[2]) {
				boolean given = !missing(initial);
				if (!given && surely == 0)
					// reduce() of an iterable that might be empty with no
					// initial value
					raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
				if (!given && length != null && length == 0)
					continue;

				// the values of acc: initial, or the first element
				AnalysisState<A> acc = given ? state.withExecutionExpressions(new ExpressionSet(initial))
						: Iteration.elements(analysis, context, state, iterableExpr, iterable, st, getMetaVariables(),
								raising);
				if (given || length == null || length != 1)
					// function is surely called if the iterable surely has an
					// element (two, without an initial value)
					acc = accumulate(interprocedural, acc, params[0], iterableExpr, iterable, expressions,
							surely >= (given ? 1 : 2));
				result = result.lub(acc);
			}
		}
		return raising.moveFrom(result);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// never invoked: forwardSemanticsAux computes the result
		throw new SemanticException("reduce is evaluated by forwardSemanticsAux");
	}

	// the values of acc after calling function on its values and any element,
	// until they are stable; if function is surely called, the initial values
	// are excluded, as the results of calling it on the stable values of acc
	// include the ones of all the calls
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> accumulate(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> initial,
			ExpressionSet function,
			Expression iterableExpr,
			SymbolicExpression iterable,
			StatementStore<A> expressions,
			boolean called)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		AnalysisState<A> acc = initial;
		for (int calls = 1;; calls++) {
			// the elements are read in the current state, that the calls
			// might have changed
			AnalysisState<A> elements = Iteration.elements(analysis, context, acc, iterableExpr, iterable, st,
					getMetaVariables());
			AnalysisState<A> results = PyValueCall.call(interprocedural, elements, function,
					new ExpressionSet[] { acc.getExecutionExpressions(), elements.getExecutionExpressions() }, st,
					"reduce", expressions, getMetaVariables());
			AnalysisState<A> next = acc.lub(results);
			if (calls > WIDEN_AFTER)
				next = acc.widening(next);
			if (next.lessOrEqual(acc))
				return called ? results : acc;
			acc = next;
		}
	}
}
