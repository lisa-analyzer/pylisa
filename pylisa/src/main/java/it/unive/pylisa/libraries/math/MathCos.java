package it.unive.pylisa.libraries.math;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * Native implementation of {@code math.cos(x)} (see {@link MathFloatFunction}):
 * an infinite {@code x} raises {@code ValueError} ({@code nan} does not).
 */
public class MathCos extends MathFloatFunction {

	protected MathCos(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "cos", params);
	}

	public static MathCos build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new MathCos(cfg, location, exprs);
	}

	@Override
	protected SymbolicExpression outsideDomain(
			SymbolicExpression x,
			CodeLocation loc) {
		return MathSin.infinite(x, loc);
	}
}
