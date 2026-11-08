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
