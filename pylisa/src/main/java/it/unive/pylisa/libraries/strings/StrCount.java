package it.unive.pylisa.libraries.strings;

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
import it.unive.pylisa.symbolic.operators.strings.StrSearch;

/**
 * Native implementation of {@code str.count(self, sub, start=None, end=None)}.
 * See {@link StrSearchSemantics}.
 */
public class StrCount extends PyNative {

	protected StrCount(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "count", params);
	}

	public static StrCount build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrCount(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return StrSearchSemantics.semantics(this, analysis, state, args, StrSearch.COUNT, null,
				false);
	}
}
