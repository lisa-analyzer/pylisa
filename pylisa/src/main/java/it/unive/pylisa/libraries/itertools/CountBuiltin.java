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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;

/**
 * Native implementation of {@code itertools.count(start=0, step=1)}: an
 * iterator over numbers (see {@link ItertoolsModel}), that is never exhausted.
 * {@code TypeError} is raised if {@code start} or {@code step} might not be
 * numbers.
 */
public class CountBuiltin extends ItertoolBuiltin {

	protected CountBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "count", params);
	}

	public static CountBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CountBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression start = missing(args[0]) ? new Constant(Int32Type.INSTANCE, 0, loc) : args[0];
		SymbolicExpression step = missing(args[1]) ? new Constant(Int32Type.INSTANCE, 1, loc) : args[1];
		Satisfiability typed = hasType(analysis, context, state, start, NUMBER)
				.and(hasType(analysis, context, state, step, NUMBER));
		return checked(analysis, context, state, typed, ItertoolsModel.COUNT,
				new String[] { ItertoolsModel.START, ItertoolsModel.STEP }, new SymbolicExpression[] { start, step });
	}
}
