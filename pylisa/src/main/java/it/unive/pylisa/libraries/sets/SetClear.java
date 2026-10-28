package it.unive.pylisa.libraries.sets;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code set.clear(self)}, that removes all the
 * elements of the set and returns {@code None}. As the elements of a set are
 * joined with the ones it had before (see {@link SetModel#add}), they are not
 * changed: the set might be any of its subsets, also the empty one.
 */
public class SetClear extends PyNative {

	protected SetClear(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "clear", params);
	}

	public static SetClear build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetClear(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state, new PyNoneConstant(getLocation()));
	}
}
