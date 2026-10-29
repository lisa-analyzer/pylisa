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
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression raiseCondition,
			String exception,
			SymbolicExpression operation,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower)
			throws SemanticException {
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		return raising.moveFrom(guardedCompute(analysis, context, state, raiseCondition, exception, operation, cfg, loc,
				dispatchTarget, thrower, raising));
	}

	/**
	 * Same as
	 * {@link #guardedCompute(Analysis, ScopeId, AnalysisState, SymbolicExpression, String, SymbolicExpression, CFG, CodeLocation, Statement, Statement)},
	 * but the exception is collected in {@code raising} instead of being moved
	 * to the error channel, for operations whose result is joined with the one
	 * of other operations.
	 *
	 * @param <A>            the kind of {@link AbstractLattice} of the state
	 * @param <D>            the kind of {@link AbstractDomain} of the analysis
	 * @param analysis       the analysis
	 * @param context        the interprocedural context where the computation
	 *                           happens
	 * @param state          the state where the operands have been evaluated
	 * @param raiseCondition the condition under which the exception is raised
	 * @param exception      the name of the class of the exception
	 * @param operation      the operation computed when the exception is not
	 *                           raised
	 * @param cfg            the cfg where the exception is raised
	 * @param loc            the location where the exception is raised
	 * @param dispatchTarget the statement computing the operation
	 * @param thrower        the statement raising the exception
	 * @param raising        where the raised exception is collected
	 *
	 * @return the state after the operation, where the executions raising the
	 *             exception have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> guardedCompute(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression raiseCondition,
			String exception,
			SymbolicExpression operation,
			CFG cfg,
			CodeLocation loc,
			Statement dispatchTarget,
			Statement thrower,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		Satisfiability sat = analysis.satisfies(state, raiseCondition, context, thrower);

		AnalysisState<A> result = state.bottomExecution();
		if (sat != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state, operation, context, dispatchTarget));
		if (sat != Satisfiability.NOT_SATISFIED)
			raising.raise(state, cfg, loc, thrower, exception);

		return result;
	}
}
