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
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
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
 * {@code key} might not be {@code None}, it is called on the elements, and the
 * code it runs is not known: it might raise any exception.
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
		return super.forwardSemanticsAux(interprocedural, created, params, expressions);
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

		AnalysisState<A> result = state.bottomExecution();
		boolean comparable = true;
		if (typed != Satisfiability.NOT_SATISFIED) {
			Expression[] subs = getSubExpressions();
			AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 0 ? subs[0] : null,
					iterable, st, getMetaVariables());
			Satisfiability numbers = Satisfiability.BOTTOM, strs = Satisfiability.BOTTOM,
					bytes = Satisfiability.BOTTOM;
			for (SymbolicExpression element : elements.getExecutionExpressions()) {
				numbers = numbers.lub(hasType(analysis, elements, element, NUMBER));
				strs = strs.lub(hasType(analysis, elements, element, STR));
				bytes = bytes.lub(hasType(analysis, elements, element, BYTES));
			}
			// no element (e.g. an empty list), or elements of a single family
			comparable = numbers == Satisfiability.BOTTOM || numbers == Satisfiability.SATISFIED
					|| strs == Satisfiability.SATISFIED || bytes == Satisfiability.SATISFIED;

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
			if (!comparable)
				result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
			if (hasType(analysis, state, key, NONE) != Satisfiability.SATISFIED)
				// the code of key is not known
				result = result.lub(raise(analysis, state, LibrarySpecificationProvider.EXCEPTION));
		}
		return typeChecked(analysis, state, typed, result);
	}
}
