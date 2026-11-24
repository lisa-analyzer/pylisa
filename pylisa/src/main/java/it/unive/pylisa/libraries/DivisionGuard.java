package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Type;
import java.util.Set;

/**
 * Shared helper for {@code __truediv__}/{@code __floordiv__}/{@code __mod__}
 * (and their reflected counterparts): checks whether the divisor is (possibly)
 * zero, raising {@code ZeroDivisionError} if so, through
 * {@link ExceptionGuard}.
 */
public final class DivisionGuard {

	private DivisionGuard() {
	}

	/**
	 * Computes the given division, raising {@code ZeroDivisionError} exactly
	 * in the executions where the divisor is zero (see
	 * {@link ExceptionGuard#guardedCompute(Analysis, ScopeId, AnalysisState, SymbolicExpression, String, SymbolicExpression, CFG, CodeLocation, Statement, Statement)}).
	 * An int divisor is compared with the int {@code 0}, also when the given
	 * zero is a float, so that the domain of ints decides the comparison.
	 *
	 * @param <A>            the kind of {@link AbstractLattice} of the state
	 * @param <D>            the kind of {@link AbstractDomain} of the analysis
	 * @param analysis       the analysis
	 * @param context        the interprocedural context where the computation
	 *                           happens
	 * @param state          the state where the operands have been evaluated
	 * @param divisor        the divisor
	 * @param zero           the zero of the type of the division
	 * @param operation      the division, computed where the divisor is not
	 *                           zero
	 * @param cfg            the cfg where the exception is raised
	 * @param loc            the location where the exception is raised
	 * @param dispatchTarget the statement computing the division
	 * @param thrower        the statement raising the exception
	 *
	 * @return the state after the division, where the executions raising the
	 *             exception have been moved to the error channel
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> guardedCompute(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression divisor,
			Constant zero,
			BinaryExpression operation,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, divisor, context, thrower);
		if (!types.isEmpty() && types.stream().allMatch(PyNative.INT))
			// an integer divisor (e.g. of a float, as in 1.0 / n) is compared
			// with the integer 0, so that its interval decides the comparison
			zero = new Constant(Int32Type.INSTANCE, 0, zero.getCodeLocation());
		BinaryExpression isZero = new BinaryExpression(BoolType.INSTANCE, divisor, zero, ComparisonEq.INSTANCE, loc);
		return ExceptionGuard.guardedCompute(analysis, context, state, isZero,
				LibrarySpecificationProvider.ZERO_DIVISION_ERROR,
				operation, cfg, loc, dispatchTarget, thrower);
	}
}
