package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
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
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.symbolic.operators.FloatPower;
import it.unive.pylisa.symbolic.operators.Power;
import java.util.Set;

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
	 * @param context        the interprocedural context where the computation
	 *                           happens
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
			ScopeId<?> context,
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

		if (!intOperands) {
			BinaryExpression negativeBase = new BinaryExpression(BoolType.INSTANCE, base, zero, ComparisonLt.INSTANCE,
					loc);
			Set<Type> exponentTypes = analysis.getRuntimeTypesOf(state, exponent, context, thrower);
			boolean integralExponent = !exponentTypes.isEmpty() && exponentTypes.stream().allMatch(PyNative.INT);
			if (integralExponent
					|| analysis.satisfies(state, negativeBase, context, thrower) == Satisfiability.NOT_SATISFIED)
				return ExceptionGuard.guardedCompute(analysis, context, state, zeroDivision,
						LibrarySpecificationProvider.ZERO_DIVISION_ERROR, floatPower, cfg, loc, dispatchTarget,
						thrower);
			// a negative number raised to a fractional power is a complex,
			// that is not modeled: the result is any value
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
			AnalysisState<A> result = state.bottomExecution();
			AnalysisState<A> nonNegative = analysis.assume(state,
					new BinaryExpression(BoolType.INSTANCE, base, zero, ComparisonGe.INSTANCE, loc), context, thrower,
					thrower);
			if (!nonNegative.getExecutionState().isBottom())
				result = ExceptionGuard.guardedCompute(analysis, context, nonNegative, zeroDivision,
						LibrarySpecificationProvider.ZERO_DIVISION_ERROR, floatPower, cfg, loc, dispatchTarget, thrower,
						raising);
			AnalysisState<A> negative = analysis.assume(state, negativeBase, context, thrower, thrower);
			if (!negative.getExecutionState().isBottom())
				result = result.lub(analysis.smallStepSemantics(negative, new PushAny(Untyped.INSTANCE, loc), context,
						dispatchTarget));
			return raising.moveFrom(result);
		}

		Satisfiability negative = analysis.satisfies(state, negativeExponent, context, thrower);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (negative != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state,
					new BinaryExpression(Int32Type.INSTANCE, base, exponent, Power.INSTANCE, loc), context,
					dispatchTarget));
		if (negative != Satisfiability.NOT_SATISFIED)
			result = result.lub(ExceptionGuard.guardedCompute(analysis, context, state, zeroDivision,
					LibrarySpecificationProvider.ZERO_DIVISION_ERROR, floatPower, cfg, loc, dispatchTarget, thrower,
					raising));
		return raising.moveFrom(result);
	}
}
