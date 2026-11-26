package it.unive.pylisa.libraries.math;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;

/**
 * Native implementation of {@code math.sin(x)} (see {@link MathFloatFunction}):
 * an infinite {@code x} raises {@code ValueError} ({@code nan} does not).
 */
public class MathSin extends MathFloatFunction {

	protected MathSin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "sin", params);
	}

	public static MathSin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new MathSin(cfg, location, exprs);
	}

	@Override
	protected SymbolicExpression outsideDomain(
			SymbolicExpression x,
			CodeLocation loc) {
		return infinite(x, loc);
	}

	/**
	 * The condition holding when {@code x} is infinite.
	 *
	 * @param x   the value
	 * @param loc the location of the condition
	 *
	 * @return the condition
	 */
	static SymbolicExpression infinite(
			SymbolicExpression x,
			CodeLocation loc) {
		return new BinaryExpression(BoolType.INSTANCE,
				new BinaryExpression(BoolType.INSTANCE, x,
						new Constant(Float64Type.INSTANCE, Double.POSITIVE_INFINITY, loc), ComparisonEq.INSTANCE, loc),
				new BinaryExpression(BoolType.INSTANCE, x,
						new Constant(Float64Type.INSTANCE, Double.NEGATIVE_INFINITY, loc), ComparisonEq.INSTANCE, loc),
				LogicalOr.INSTANCE, loc);
	}
}
