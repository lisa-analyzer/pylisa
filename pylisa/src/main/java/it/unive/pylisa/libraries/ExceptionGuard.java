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
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * Shared helper for operations that raise a Python exception under some
 * condition on their operands (e.g. {@code ValueError} for a negative shift
 * count): it checks whether the condition (possibly) holds via
 * {@code Analysis#satisfies}, and either evaluates {@code operation} (when the
 * condition might not hold) or raises the exception (via {@link PyExceptions},
 * when it might hold), {@code lub}-ing both branches together when the
 * condition cannot be decided.
 */
public final class ExceptionGuard {

	private ExceptionGuard() {
	}

	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> guardedCompute(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression raiseCondition,
			String exception,
			SymbolicExpression operation,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower)
			throws SemanticException {
		Satisfiability sat = analysis.satisfies(state, raiseCondition, thrower);

		AnalysisState<A> result = state.bottom();
		if (sat != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state, operation, dispatchTarget));
		if (sat != Satisfiability.NOT_SATISFIED)
			result = result.lub(PyExceptions.raise(analysis, state, cfg, loc, thrower, exception));

		return result;
	}
}
