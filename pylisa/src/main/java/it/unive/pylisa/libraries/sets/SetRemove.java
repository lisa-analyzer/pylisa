package it.unive.pylisa.libraries.sets;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code set.remove(self, elem)}, that removes
 * {@code elem} from the set and returns {@code None}, raising {@code KeyError}
 * if the set does not contain it: as the elements are not tracked precisely,
 * {@code KeyError} might always be raised (and surely, if the set has no
 * elements). The elements are not changed, as the removed one might still be
 * another element. Whether {@code elem} is hashable is not checked.
 */
public class SetRemove extends PyNative {

	protected SetRemove(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "remove", params);
	}

	public static SetRemove build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetRemove(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		boolean empty = SetModel.elements(analysis, context, state, args[0], st).getExecutionState().isBottom();
		if (!empty)
			result = compute(analysis, context, state, new PyNoneConstant(getLocation()));
		// KeyError if the element is not in the set: surely if the set has no
		// elements, and under an unknown condition otherwise
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		raise(raising, empty ? Satisfiability.SATISFIED : Satisfiability.UNKNOWN, state,
				LibrarySpecificationProvider.KEY_ERROR);
		return raising.moveFrom(result);
	}
}
