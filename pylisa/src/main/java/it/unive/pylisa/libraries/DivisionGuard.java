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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;

/**
 * Shared helper for {@code __truediv__}/{@code __floordiv__}/{@code __mod__}
 * (and their reflected counterparts): checks whether the divisor is (possibly)
 * zero via {@code Analysis#satisfies} &mdash; mirroring how
 * {@code SequenceGetItem} bounds-checks a tuple index &mdash; and either
 * evaluates {@code operation} (when the divisor might be nonzero) or raises
 * {@code ZeroDivisionError} (via {@link PyExceptions}, when it might be zero),
 * {@code lub}-ing both branches together when the divisor's value is not
 * statically known.
 */
public final class DivisionGuard {

	private DivisionGuard() {
	}

	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> guardedCompute(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression divisor,
			Constant zero,
			BinaryExpression operation,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower)
			throws SemanticException {
		BinaryExpression isZero = new BinaryExpression(BoolType.INSTANCE, divisor, zero, ComparisonEq.INSTANCE, loc);
		Satisfiability sat = analysis.satisfies(state, isZero, thrower);

		AnalysisState<A> result = state.bottom();
		if (sat != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state, operation, dispatchTarget));
		if (sat != Satisfiability.NOT_SATISFIED)
			result = result.lub(PyExceptions.raise(analysis, state, cfg, loc, thrower,
					LibrarySpecificationProvider.ZERO_DIVISION_ERROR));

		return result;
	}
}
