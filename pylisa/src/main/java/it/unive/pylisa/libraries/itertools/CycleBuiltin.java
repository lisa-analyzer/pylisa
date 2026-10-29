package it.unive.pylisa.libraries.itertools;

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
import it.unive.pylisa.cfg.expression.Iteration;

/**
 * Native implementation of {@code itertools.cycle(iterable)}: an iterator over
 * the elements of {@code iterable}, repeated forever (see
 * {@link ItertoolsModel}). {@code TypeError} is raised if {@code iterable}
 * might not be iterable (see {@link Iteration#iterable}).
 */
public class CycleBuiltin extends ItertoolBuiltin {

	protected CycleBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "cycle", params);
	}

	public static CycleBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CycleBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return checked(analysis, context, state, Iteration.iterable(analysis, context, state, args[0], st),
				ItertoolsModel.CYCLE,
				new String[] { ItertoolsModel.ITERABLE }, new SymbolicExpression[] { args[0] });
	}
}
