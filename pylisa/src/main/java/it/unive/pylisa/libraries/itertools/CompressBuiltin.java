package it.unive.pylisa.libraries.itertools;

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

/**
 * Native implementation of {@code itertools.compress(data, selectors)}: an
 * iterator over the elements of {@code data} whose selector is true (see
 * {@link ItertoolsModel}). {@code TypeError} is raised if {@code data} or
 * {@code selectors} might not be iterable (see {@link Iteration#iterable}).
 */
public class CompressBuiltin extends ItertoolBuiltin {

	protected CompressBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "compress", params);
	}

	public static CompressBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CompressBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Satisfiability typed = Iteration.iterable(analysis, state, args[0], st)
				.and(Iteration.iterable(analysis, state, args[1], st));
		return checked(analysis, state, typed, ItertoolsModel.COMPRESS, new String[] { ItertoolsModel.ITERABLE },
				new SymbolicExpression[] { args[0] });
	}
}
