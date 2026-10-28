package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
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
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code list.extend(self, iterable)}, that adds the
 * elements of {@code iterable} at the end of the list and returns {@code None}.
 * As for {@code append} (see {@link ListAppend}), the elements are added to the
 * elements at unknown indices of the list (a weak update), and the elements at
 * known indices are not changed. {@code TypeError} is raised if
 * {@code iterable} might not be iterable (see {@link Iteration#iterable}).
 */
public class ListExtend extends PyNative {

	protected ListExtend(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "extend", params);
	}

	public static ListExtend build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListExtend(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability iterable = Iteration.iterable(analysis, state, args[1], st);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		if (iterable != Satisfiability.NOT_SATISFIED) {
			Expression[] subs = getSubExpressions();
			AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 1 ? subs[1] : null,
					args[1], st, getMetaVariables(), raising);
			Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
			AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, args[0], loc),
					new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
			// weak update: the new elements are joined with the previous ones
			// (the iterable might also be empty)
			AnalysisState<A> extended = state;
			for (SymbolicExpression element : elements.getExecutionExpressions())
				extended = extended.lub(analysis.assign(elements, unknown, element, st));
			result = compute(analysis, extended, new PyNoneConstant(loc));
		}
		if (iterable != Satisfiability.SATISFIED)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
