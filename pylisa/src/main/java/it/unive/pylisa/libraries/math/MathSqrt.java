package it.unive.pylisa.libraries.math;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;

/**
 * Native implementation of {@code math.sqrt(x)} (see
 * {@link MathFloatFunction}): a negative {@code x} raises {@code ValueError}
 * ({@code nan} does not).
 */
public class MathSqrt extends MathFloatFunction {

	protected MathSqrt(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "sqrt", params);
	}

	public static MathSqrt build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new MathSqrt(cfg, location, exprs);
	}

	@Override
	protected SymbolicExpression outsideDomain(
			SymbolicExpression x,
			CodeLocation loc) {
		return new BinaryExpression(BoolType.INSTANCE, x, new Constant(Int32Type.INSTANCE, 0, loc),
				ComparisonLt.INSTANCE, loc);
	}
}
