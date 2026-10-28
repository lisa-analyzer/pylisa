package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyValueCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.function.Predicate;

/**
 * Native implementation of
 * {@code sorted(iterable, *, key=None, reverse=False)}: a new list with the
 * elements of {@code iterable} at unknown indices (see
 * {@link ListCreation#UNKNOWN_INDEX}), whose order and length are not tracked.
 * {@code TypeError} is raised if {@code iterable} might not be iterable (see
 * {@link Iteration#iterable}), if {@code reverse} might not be an {@code int},
 * or if the elements might not be compared: they are all numbers, all
 * {@code str}s, or all {@code bytes}, otherwise comparing two of them might
 * raise {@code TypeError} (e.g. an {@code int} and a {@code str}). If
 * {@code key} might not be {@code None}, it is called on the elements (see
 * {@link PyValueCall#call}), raising the exceptions it might raise, and its
 * results are compared instead of the elements; if it is not known whether it
 * can be called, any exception might be raised.
 */
public class SortedBuiltin extends PyNative {

	// the families of values that can be compared with each other
	private static final Predicate<Type> NUMBER = t -> t.isNumericType() || t.isBooleanType();

	protected SortedBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "sorted", params);
	}

	public static SortedBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SortedBuiltin(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// the new list: the state of the arguments is the one where it has
		// been created, whose computed expressions are the list (see
		// semantics)
		ListCreation creation = new ListCreation(getCFG(), getLocation());
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		AnalysisState<A> result = super.forwardSemanticsAux(interprocedural, created, params, expressions);

		// key is called on the elements, and its results are compared
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		Expression[] subs = getSubExpressions();
		for (SymbolicExpression key : params[1]) {
			if (hasType(analysis, created, key, NONE) == Satisfiability.SATISFIED)
				continue;
			if (PyValueCall.callable(analysis, created, key, st) == Satisfiability.UNKNOWN)
				// the code of key is not known
				raise(raising, created, LibrarySpecificationProvider.EXCEPTION);
			for (SymbolicExpression iterable : params[0]) {
				if (Iteration.iterable(analysis, created, iterable, st) == Satisfiability.NOT_SATISFIED)
					continue;
				AnalysisState<A> elements = Iteration.elements(interprocedural, created,
						subs.length > 0 ? subs[0] : null, iterable, st, getMetaVariables());
				AnalysisState<A> keys = PyValueCall.call(interprocedural, elements, new ExpressionSet(key),
						new ExpressionSet[] { elements.getExecutionExpressions() }, st, "key",
						new StatementStore<>(elements), getMetaVariables());
				// the exceptions raised by key
				result = result.lub(keys.bottomExecution());
				if (!comparable(analysis, keys, keys.getExecutionExpressions()))
					raise(raising, created, LibrarySpecificationProvider.TYPE_ERROR);
			}
		}
		return raising.moveFrom(result);
	}

	// whether the values can be compared with each other: there are none, or
	// they are all numbers, all str or all bytes
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean comparable(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet values)
			throws SemanticException {
		Satisfiability numbers = Satisfiability.BOTTOM, strs = Satisfiability.BOTTOM, bytes = Satisfiability.BOTTOM;
		for (SymbolicExpression value : values) {
			numbers = numbers.lub(hasType(analysis, state, value, NUMBER));
			strs = strs.lub(hasType(analysis, state, value, STR));
			bytes = bytes.lub(hasType(analysis, state, value, BYTES));
		}
		return numbers == Satisfiability.BOTTOM || numbers == Satisfiability.SATISFIED
				|| strs == Satisfiability.SATISFIED || bytes == Satisfiability.SATISFIED;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression iterable = args[0], key = args[1], reverse = args[2];
		Satisfiability typed = Iteration.iterable(analysis, state, iterable, st);
		if (!missing(reverse))
			typed = typed.and(hasType(analysis, state, reverse, INT));

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			Expression[] subs = getSubExpressions();
			AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 0 ? subs[0] : null,
					iterable, st, getMetaVariables(), raising);

			// the elements at unknown indices of the new list
			Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
			AnalysisState<A> filled = state;
			for (SymbolicExpression list : state.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				for (SymbolicExpression element : elements.getExecutionExpressions())
					filled = filled.lub(analysis.assign(elements, unknown, element, st)
							.withExecutionExpression(list));
			}
			result = filled;
			if (hasType(analysis, state, key, NONE) != Satisfiability.NOT_SATISFIED
					&& !comparable(analysis, elements, elements.getExecutionExpressions()))
				// without key, the elements are compared (with key, its results
				// are, see forwardSemanticsAux)
				raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		}
		return raising.moveFrom(typeChecked(state, typed, result, raising));
	}
}
