package it.unive.pylisa.libraries.sets;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code set.pop(self)}, that removes and returns any
 * element of the set, raising {@code KeyError} if the set is empty: as the
 * number of elements is not tracked, {@code KeyError} might always be raised
 * (and surely, if the set has no elements). The elements are not changed, as
 * the removed one might still be another element.
 */
public class SetPop extends PyNative {

	protected SetPop(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "pop", params);
	}

	public static SetPop build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetPop(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// KeyError if the set is empty
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		raise(raising, state, LibrarySpecificationProvider.KEY_ERROR);
		return raising.moveFrom(SetModel.elements(analysis, context, state, args[0], st));
	}
}
