package it.unive.pylisa.libraries.itertools;

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

/**
 * Native implementation of {@code itertools.chain(*iterables)}: an iterator
 * over the elements of the iterables, in order (see {@link ItertoolsModel}). As
 * in python, the iterables are iterated (raising {@code TypeError} if they are
 * not iterable) when the iterator is.
 */
public class ChainBuiltin extends ItertoolBuiltin {

	protected ChainBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "chain", params);
	}

	public static ChainBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ChainBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return checked(analysis, context, state, Satisfiability.SATISFIED, ItertoolsModel.CHAIN,
				new String[] { ItertoolsModel.ITERABLES }, new SymbolicExpression[] { args[0] });
	}
}
