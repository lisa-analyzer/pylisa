package it.unive.pylisa.libraries.itertools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.Iteration;

/**
 * Native implementation of {@code itertools.combinations(iterable, r)}: an
 * iterator over the tuples of {@code r} elements of {@code iterable} (see
 * {@link ItertoolsModel}). {@code TypeError} is raised if {@code iterable}
 * might not be iterable (see {@link Iteration#iterable}), or if {@code r} might
 * not be an {@code int}; {@code ValueError} might be raised if {@code r} might
 * be negative.
 */
public class CombinationsBuiltin extends ItertoolBuiltin {

	protected CombinationsBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "combinations", params);
	}

	public static CombinationsBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CombinationsBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return tuples(analysis, state, args, ItertoolsModel.COMBINATIONS, false);
	}
}
