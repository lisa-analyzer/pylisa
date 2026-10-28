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
 * Native implementation of {@code itertools.permutations(iterable, r=None)}: an
 * iterator over the tuples of {@code r} elements of {@code iterable} (see
 * {@link ItertoolsModel}). {@code TypeError} is raised if {@code iterable}
 * might not be iterable (see {@link Iteration#iterable}), or if {@code r} might
 * be neither an {@code int} nor {@code None}; {@code ValueError} might be
 * raised if {@code r} might be negative.
 */
public class PermutationsBuiltin extends ItertoolBuiltin {

	protected PermutationsBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "permutations", params);
	}

	public static PermutationsBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new PermutationsBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return tuples(analysis, state, args, ItertoolsModel.PERMUTATIONS, true);
	}
}
