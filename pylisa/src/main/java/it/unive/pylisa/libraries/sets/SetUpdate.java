package it.unive.pylisa.libraries.sets;

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
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code set.update(self, *others)}, that adds the
 * elements of the iterables {@code others} to the set (see
 * {@link Iteration#elements} and {@link SetModel#add}) and returns
 * {@code None}. {@code TypeError} is raised if an iterable might not be
 * iterable (see {@link Iteration#iterable}); if the number of the iterables is
 * not known, any element might be added, and {@code TypeError} might be raised.
 * Whether the elements are hashable is not checked.
 */
public class SetUpdate extends PyNative {

	protected SetUpdate(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "update", params);
	}

	public static SetUpdate build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetUpdate(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression self = args[0], others = args[1];
		java.util.List<SymbolicExpression> iterables = ListModel.exactElements(analysis, state, others, st);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		if (iterables == null) {
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
			return raising.moveFrom(compute(analysis, SetModel.invalidate(analysis, state, self, st),
					new PyNoneConstant(getLocation())));
		}
		AnalysisState<A> current = state;
		boolean typeError = false;
		for (SymbolicExpression iterable : iterables) {
			Satisfiability it = Iteration.iterable(analysis, current, iterable, st);
			typeError |= it != Satisfiability.SATISFIED;
			if (it == Satisfiability.NOT_SATISFIED) {
				current = current.bottomExecution();
				break;
			}
			AnalysisState<A> elements = Iteration.elements(analysis, current, null, iterable, st,
					getMetaVariables(), raising);
			AnalysisState<A> next = elements.getExecutionState().isBottom() ? current : current.bottomExecution();
			for (SymbolicExpression element : elements.getExecutionExpressions())
				next = next.lub(SetModel.add(analysis, elements, self, element, st));
			current = next;
		}
		AnalysisState<A> result = compute(analysis, current, new PyNoneConstant(getLocation()));
		if (typeError)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
