package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.pylisa.symbolic.operators.FloatPower;
import it.unive.pylisa.symbolic.operators.Power;

/**
 * Shared semantics of {@code __pow__}/{@code __rpow__} for {@code int} and
 * {@code float}, mirroring Python:
 * <ul>
 * <li>{@code int ** int} is an {@code int} if the exponent is non-negative, and
 * a {@code float} otherwise ({@code 2 ** -1 == 0.5});</li>
 * <li>if either operand is a {@code float}, the result is a {@code float};</li>
 * <li>raising {@code 0} (or {@code 0.0}) to a negative power raises
 * {@code ZeroDivisionError}.</li>
 * </ul>
 * Raising a negative number to a fractional power yields a {@code complex} in
 * Python, which is not modeled: the domains are expected to return an unknown
 * value for it.
 */
public final class PowerSemantics {

	private PowerSemantics() {
	}

	/**
	 * Computes {@code base ** exponent}.
	 *
	 * @param analysis       the analysis
	 * @param state          the state where both operands have been evaluated
	 * @param base           the base
	 * @param exponent       the exponent
	 * @param intOperands    whether both operands are {@code int}s
	 * @param cfg            the cfg where the operation happens
	 * @param loc            the location of the operation
	 * @param dispatchTarget the statement the operation is attributed to
	 * @param thrower        the statement raising the exceptions
	 *
	 * @return the state after the operation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> compute(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression base,
			SymbolicExpression exponent,
			boolean intOperands,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower)
			throws SemanticException {
		Constant zero = new Constant(Int32Type.INSTANCE, 0, loc);
		BinaryExpression negativeExponent = new BinaryExpression(BoolType.INSTANCE, exponent, zero,
				ComparisonLt.INSTANCE, loc);
		BinaryExpression zeroBase = new BinaryExpression(BoolType.INSTANCE, base, zero, ComparisonEq.INSTANCE, loc);
		BinaryExpression zeroDivision = new BinaryExpression(BoolType.INSTANCE, zeroBase, negativeExponent,
				LogicalAnd.INSTANCE, loc);
		BinaryExpression floatPower = new BinaryExpression(Float32Type.INSTANCE, base, exponent,
				FloatPower.INSTANCE, loc);

		if (!intOperands)
			return ExceptionGuard.guardedCompute(analysis, state, zeroDivision,
					LibrarySpecificationProvider.ZERO_DIVISION_ERROR, floatPower, cfg, loc, dispatchTarget, thrower);

		Satisfiability negative = analysis.satisfies(state, negativeExponent, thrower);
		AnalysisState<A> result = state.bottom();
		if (negative != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state,
					new BinaryExpression(Int32Type.INSTANCE, base, exponent, Power.INSTANCE, loc), dispatchTarget));
		if (negative != Satisfiability.NOT_SATISFIED)
			result = result.lub(ExceptionGuard.guardedCompute(analysis, state, zeroDivision,
					LibrarySpecificationProvider.ZERO_DIVISION_ERROR, floatPower, cfg, loc, dispatchTarget, thrower));
		return result;
	}
}
