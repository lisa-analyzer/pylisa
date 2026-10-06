package it.unive.pylisa.libraries.conversions;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.conversions.ToRepr;

/**
 * Native implementation of {@code repr(x)}.
 */
public class ReprBuiltin extends PyNative {

	protected ReprBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "repr", params);
	}

	public static ReprBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReprBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new UnaryExpression(StringType.INSTANCE, args[0], ToRepr.INSTANCE, getLocation()));
	}
}
