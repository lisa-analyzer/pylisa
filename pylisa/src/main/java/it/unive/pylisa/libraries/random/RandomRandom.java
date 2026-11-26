package it.unive.pylisa.libraries.random;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code random.random()}: a {@code float} (in
 * {@code [0, 1)}, which is not tracked).
 */
public class RandomRandom extends PyNative {

	protected RandomRandom(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "random", params);
	}

	public static RandomRandom build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RandomRandom(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return unknown(analysis, context, state, Float64Type.INSTANCE);
	}
}
