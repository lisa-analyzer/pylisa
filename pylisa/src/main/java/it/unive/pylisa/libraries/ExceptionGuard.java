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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;

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

		// when the condition is not decided, each branch is restricted to the
		// executions it describes: the operation is computed where the
		// condition does not hold, and the exception is raised where it does
		// (so that, e.g., the verification of the warnings sees under which
		// condition the exception is raised)
		AnalysisState<A> result = state.bottomExecution();
		if (sat != Satisfiability.SATISFIED) {
			AnalysisState<A> computing = sat == Satisfiability.NOT_SATISFIED ? state
					: analysis.assume(state, new UnaryExpression(BoolType.INSTANCE, raiseCondition,
							LogicalNegation.INSTANCE, loc), context, thrower, thrower);
			if (!computing.getExecutionState().isBottom())
				result = result.lub(analysis.smallStepSemantics(computing, operation, context, dispatchTarget));
		}
		if (sat != Satisfiability.NOT_SATISFIED) {
			AnalysisState<A> raisingState = sat == Satisfiability.SATISFIED ? state
					: analysis.assume(state, raiseCondition, context, thrower, thrower);
			if (!raisingState.getExecutionState().isBottom())
				raising.raise(Satisfiability.SATISFIED, raisingState, cfg, loc, thrower, exception);
		}

		return result;
	}

	/**
	 * Restricts the given state to the executions where the given boolean
	 * condition holds, for the executions of a branch raising an exception (or
	 * computing a result) only under that condition. This makes the condition
	 * of the branch part of the state, e.g., for the verification of the
	 * warnings, that otherwise sees the exception raised in every execution.
	 *
	 * @param <A>       the kind of {@link AbstractLattice} of the state
	 * @param <D>       the kind of {@link AbstractDomain} of the analysis
	 * @param analysis  the analysis
	 * @param context   the interprocedural context
	 * @param state     the state
	 * @param condition the condition of the branch
	 * @param pp        the program point of the branch
	 *
	 * @return the restricted state
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> when(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression condition,
			Statement pp)
			throws SemanticException {
		return analysis.assume(state, condition, context, pp, pp);
	}

	/**
	 * Restricts the given state to the executions where the given boolean
	 * condition does not hold (see
	 * {@link #when(Analysis, ScopeId, AnalysisState, SymbolicExpression, Statement)}).
	 *
	 * @param <A>       the kind of {@link AbstractLattice} of the state
	 * @param <D>       the kind of {@link AbstractDomain} of the analysis
	 * @param analysis  the analysis
	 * @param context   the interprocedural context
	 * @param state     the state
	 * @param condition the condition that does not hold in the branch
	 * @param pp        the program point of the branch
	 *
	 * @return the restricted state
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unless(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression condition,
			Statement pp)
			throws SemanticException {
		return analysis.assume(state,
				new UnaryExpression(BoolType.INSTANCE, condition, LogicalNegation.INSTANCE, pp.getLocation()),
				context, pp, pp);
	}

	/**
	 * Restricts the given state to the executions of a branch whose condition
	 * is not expressed (e.g., an exception raised when an operand might have an
	 * unsupported type, or when a complex condition on the operands might
	 * hold): the branch happens under an unknown condition. The analysis is not
	 * affected (the condition is any boolean), while the verification of the
	 * warnings sees that the branch might not happen, and never takes it for
	 * granted (the unknown value does not model an input of the program).
	 *
	 * @param <A>      the kind of {@link AbstractLattice} of the state
	 * @param <D>      the kind of {@link AbstractDomain} of the analysis
	 * @param analysis the analysis
	 * @param context  the interprocedural context
	 * @param state    the state
	 * @param pp       the program point of the branch
	 *
	 * @return the restricted state
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> maybe(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			Statement pp)
			throws SemanticException {
		return analysis.assume(state, new PushAny(BoolType.INSTANCE, pp.getLocation()), context, pp, pp);
	}
}
