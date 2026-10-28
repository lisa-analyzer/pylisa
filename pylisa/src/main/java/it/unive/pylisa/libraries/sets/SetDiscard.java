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
 * Native implementation of {@code set.discard(self, elem)}, that removes
 * {@code elem} from the set if it contains it, and returns {@code None}. The
 * elements are not changed, as the removed one might still be another element.
 * Whether {@code elem} is hashable is not checked.
 */
public class SetDiscard extends PyNative {

	protected SetDiscard(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "discard", params);
	}

	public static SetDiscard build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetDiscard(cfg, location, exprs);
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
