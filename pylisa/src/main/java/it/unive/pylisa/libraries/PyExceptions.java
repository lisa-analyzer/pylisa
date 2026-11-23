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
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.symbolic.CFGThrow;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.CallSites;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Shared helper to raise a native Python exception from a library
 * implementation, mirroring how JLiSA raises {@code ArrayIndexOutOfBounds
 * Exception} in {@code JavaArrayAccess}: it allocates an instance of the given
 * (built-in, field-less) exception class, wraps it in a {@link CFGThrow}, and
 * moves the resulting state to the error channel. The move is <i>total</i> (see
 * {@link Analysis#moveExecutionToError(AnalysisState, AnalysisState.Error, ScopeId, ProgramPoint)})
 * when all executions raise the exception, as in
 * {@link #raise(Analysis, ScopeId, AnalysisState, CFG, CodeLocation, Statement, String)},
 * whose returned state has an empty normal execution. When only some executions
 * raise it, the move must instead be <i>partial</i> (see
 * {@link Analysis#moveExecutionToError(AnalysisState, AnalysisState, AnalysisState.Error, ScopeId, ProgramPoint)}),
 * knowing all the executions that continue normally: {@link Raising} collects
 * the raising executions until these are known.
 */
public final class PyExceptions {

	private PyExceptions() {
	}

	/**
	 * Raises the given exception in all executions of the given state
	 * (<i>total</i> move): the returned state has an empty normal execution.
	 * Use {@link Raising} if only some executions raise it.
	 *
	 * @param <A>                the kind of {@link AbstractLattice} of the
	 *                               state
	 * @param <D>                the kind of {@link AbstractDomain} of the
	 *                               analysis
	 * @param analysis           the analysis
	 * @param context            the interprocedural context where the
	 *                               computation happens
	 * @param state              the state of the executions raising the
	 *                               exception
	 * @param cfg                the cfg where the exception is raised
	 * @param loc                the location where the exception is raised
	 * @param thrower            the statement raising the exception
	 * @param exceptionClassName the name of the class of the exception
	 *
	 * @return the state where the exception has been raised
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> raise(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			CFG cfg,
			CodeLocation loc,
			Statement thrower,
			String exceptionClassName)
			throws SemanticException {
		Pair<AnalysisState.Error, AnalysisState<A>> raising = prepare(analysis, context, state, cfg, loc, thrower,
				exceptionClassName);
		AnalysisState.Error error = raising.getLeft();
		return analysis.moveExecutionToError(raising.getRight(), error, context, error.getThrower());
	}

	// yields the error raised by the given exception and the state of the
	// executions raising it, where the exception is the execution expression
	// and the execution has not been moved yet
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Pair<AnalysisState.Error,
			AnalysisState<A>> prepare(
					Analysis<A, D> analysis,
					ScopeId<?> context,
					AnalysisState<A> state,
					CFG cfg,
					CodeLocation loc,
					Statement thrower,
					String exceptionClassName)
					throws SemanticException {
		Type exceptionType = PyClassType.lookup(exceptionClassName);
		thrower = programStatement(thrower);

		// the exception has a location of its own, as the allocations are
		// identified by their location: the one of the thrower might be also
		// the one of another object (e.g. the list in [1][2])
		MemoryAllocation alloc = new MemoryAllocation(exceptionType, new DerivedLocation(loc, exceptionClassName),
				false);
		AnalysisState<A> allocState = analysis.smallStepSemantics(state, alloc, context, thrower);

		AnalysisState<A> exceptionState = state.bottomExecution();
		for (SymbolicExpression th : allocState.getExecutionExpressions()) {
			CFGThrow throwVar = new CFGThrow(cfg, exceptionType, loc);
			AnalysisState<A> tmp = analysis.assign(allocState, throwVar, th, context, thrower);
			exceptionState = exceptionState.lub(tmp.withExecutionExpression(throwVar));
		}

		return Pair.of(new AnalysisState.Error(exceptionType, thrower), exceptionState);
	}

	/**
	 * Collects the executions raising exceptions in a statement that only
	 * sometimes raises them (e.g. depending on the runtime types of its
	 * operands), so that they can be moved to the error channel with a
	 * <i>partial</i> move once all the executions that continue normally are
	 * known: exceptions are registered with
	 * {@link #raise(Satisfiability, AnalysisState, CFG, CodeLocation, Statement, String)}
	 * (or the helpers built on it, such as
	 * {@link #raiseWhere(SymbolicExpression, boolean, AnalysisState, Statement, CFG, CodeLocation, Statement, String)}),
	 * and {@link #moveFrom(AnalysisState)} performs the moves. If no execution
	 * continues normally, the moves fall back to <i>total</i> ones.
	 *
	 * @param <A> the kind of {@link AbstractLattice} of the states
	 * @param <D> the kind of {@link AbstractDomain} of the analysis
	 */
	public static final class Raising<A extends AbstractLattice<A>, D extends AbstractDomain<A>> {

		private final Analysis<A, D> analysis;

		private final ScopeId<?> context;

		// the state of the executions raising each error, not moved yet
		private final Map<AnalysisState.Error, AnalysisState<A>> raised = new LinkedHashMap<>();

		/**
		 * Builds the collector.
		 *
		 * @param analysis the analysis
		 * @param context  the interprocedural context where the exceptions are
		 *                     raised
		 */
		public Raising(
				Analysis<A, D> analysis,
				ScopeId<?> context) {
			this.analysis = analysis;
			this.context = context;
		}

		/**
		 * Raises the given exception in the executions of the given state where
		 * {@code operand} has one of the {@code failing} types, among its
		 * runtime types {@code types}: in all of them if all its types fail, in
		 * the ones where it has the failing types otherwise (see
		 * {@link PyBinaryDispatch#ofType}), under an unknown condition if they
		 * cannot be told apart from the other ones (see
		 * {@link PyBinaryDispatch#checkable}) or if {@code surely} does not
		 * hold.
		 *
		 * @param state              the state of the executions that might
		 *                               raise the exception
		 * @param operand            the value whose type raises the exception
		 * @param failing            the types of the operand raising the
		 *                               exception
		 * @param types              all the runtime types of the operand
		 * @param surely             whether the values of the failing types
		 *                               surely raise the exception
		 * @param cfg                the cfg where the exception is raised
		 * @param loc                the location where the exception is raised
		 * @param thrower            the statement raising the exception
		 * @param exceptionClassName the name of the class of the exception
		 *
		 * @throws SemanticException if the analysis fails
		 */
		public void raiseOfTypes(
				AnalysisState<A> state,
				SymbolicExpression operand,
				Set<Type> failing,
				Set<Type> types,
				boolean surely,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			raiseOfTypes(state, operand, failing, types, surely, Collections.emptySet(), cfg, loc, thrower,
					exceptionClassName);
		}

		/**
		 * As
		 * {@link #raiseOfTypes(AnalysisState, SymbolicExpression, Set, Set, boolean, CFG, CodeLocation, Statement, String)},
		 * forgetting the given identifiers (e.g. the meta variables holding the
		 * operand, consumed by the statement) in the executions raising the
		 * exception, after they are restricted to the ones where the operand
		 * has the failing types.
		 *
		 * @param state              the state of the executions that might
		 *                               raise the exception
		 * @param operand            the value whose type raises the exception
		 * @param failing            the types of the operand raising the
		 *                               exception
		 * @param types              all the runtime types of the operand
		 * @param surely             whether the values of the failing types
		 *                               surely raise the exception
		 * @param consumed           the identifiers to forget
		 * @param cfg                the cfg where the exception is raised
		 * @param loc                the location where the exception is raised
		 * @param thrower            the statement raising the exception
		 * @param exceptionClassName the name of the class of the exception
		 *
		 * @throws SemanticException if the analysis fails
		 */
		public void raiseOfTypes(
				AnalysisState<A> state,
				SymbolicExpression operand,
				Set<Type> failing,
				Set<Type> types,
				boolean surely,
				Collection<Identifier> consumed,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			if (failing.isEmpty())
				return;
			Satisfiability raises;
			AnalysisState<A> typed;
			if (types.isEmpty() || failing.stream().anyMatch(Type::isUntyped)) {
				raises = Satisfiability.UNKNOWN;
				typed = state;
			} else if (failing.containsAll(types)) {
				raises = surely ? Satisfiability.SATISFIED : Satisfiability.UNKNOWN;
				typed = state;
			} else {
				boolean checkable = surely;
				typed = state.bottomExecution();
				for (Type t : failing) {
					checkable &= PyBinaryDispatch.checkable(t, types);
					typed = typed.lub(PyBinaryDispatch.ofType(analysis, context, state, operand, t, types, thrower));
				}
				raises = checkable ? Satisfiability.SATISFIED : Satisfiability.UNKNOWN;
			}
			if (!consumed.isEmpty())
				typed = typed.forgetIdentifiers(consumed, thrower);
			raise(raises, typed, cfg, loc, thrower, exceptionClassName);
		}

		// raises the exception in all the executions of the state
		private void collect(
				AnalysisState<A> state,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			Pair<AnalysisState.Error, AnalysisState<A>> raising = prepare(analysis, context, state, cfg, loc, thrower,
					exceptionClassName);
			AnalysisState<A> previous = raised.get(raising.getLeft());
			raised.put(raising.getLeft(),
					previous == null ? raising.getRight() : previous.lub(raising.getRight()));
		}

		/**
		 * Raises the given exception in the executions of the given state where
		 * it is raised, according to {@code raises}: in none of them if it is
		 * {@link Satisfiability#NOT_SATISFIED}, in all of them if it is
		 * {@link Satisfiability#SATISFIED}, and under an unknown condition
		 * otherwise (see {@link ExceptionGuard#maybe}), so that the executions
		 * raising it are not taken as all the executions of the state when the
		 * condition raising it is not known (e.g. when the warnings are
		 * verified). {@link Satisfiability#SATISFIED} is thus only for states
		 * restricted to the executions raising the exception, and every call
		 * states which one it is.
		 *
		 * @param raises             whether the exception is raised
		 * @param state              the state of the executions that might
		 *                               raise the exception
		 * @param cfg                the cfg where the exception is raised
		 * @param loc                the location where the exception is raised
		 * @param thrower            the statement raising the exception
		 * @param exceptionClassName the name of the class of the exception
		 *
		 * @throws SemanticException if the analysis fails
		 */
		public void raise(
				Satisfiability raises,
				AnalysisState<A> state,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			if (raises == Satisfiability.NOT_SATISFIED || raises == Satisfiability.BOTTOM)
				return;
			AnalysisState<A> raising = raises == Satisfiability.SATISFIED ? state
					: ExceptionGuard.maybe(analysis, context, state, thrower);
			if (!raising.getExecutionState().isBottom())
				collect(raising, cfg, loc, thrower, exceptionClassName);
		}

		/**
		 * Raises the given exception exactly in the executions of the given
		 * state where the given condition holds, yielding the ones where it
		 * does not, that continue normally. The analysis decides whether the
		 * condition holds (see {@link Analysis#satisfies}): if it surely does,
		 * the exception is raised in all the executions; if it surely does
		 * not, in none of them; otherwise, the state is restricted to the
		 * executions where it holds to raise the exception (see
		 * {@link ExceptionGuard#when}), and to the ones where it does not to
		 * continue (see {@link ExceptionGuard#unless}). The executions where
		 * the condition holds surely raise the exception only if
		 * {@code surely} holds (e.g. not if an operand might also have a type
		 * raising another exception), otherwise the exception is raised in
		 * them under an unknown condition.
		 *
		 * @param condition          the condition under which the exception
		 *                               is raised
		 * @param surely             whether the executions where the
		 *                               condition holds surely raise the
		 *                               exception
		 * @param state              the state of the executions that might
		 *                               raise the exception
		 * @param pp                 the program point where the condition is
		 *                               checked
		 * @param cfg                the cfg where the exception is raised
		 * @param loc                the location where the exception is raised
		 * @param thrower            the statement raising the exception
		 * @param exceptionClassName the name of the class of the exception
		 *
		 * @return the state of the executions where the exception is not
		 *             raised
		 *
		 * @throws SemanticException if the analysis fails
		 */
		public AnalysisState<A> raiseWhere(
				SymbolicExpression condition,
				boolean surely,
				AnalysisState<A> state,
				Statement pp,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			if (state.getExecutionState().isBottom())
				return state;
			Satisfiability sat = analysis.satisfies(state, condition, context, pp);
			if (sat == Satisfiability.NOT_SATISFIED || sat == Satisfiability.BOTTOM)
				return state;
			Satisfiability raises = surely ? Satisfiability.SATISFIED : Satisfiability.UNKNOWN;
			if (sat == Satisfiability.SATISFIED) {
				raise(raises, state, cfg, loc, thrower, exceptionClassName);
				return state.bottomExecution();
			}
			AnalysisState<A> failing = ExceptionGuard.when(analysis, context, state, condition, pp);
			if (!failing.getExecutionState().isBottom())
				raise(raises, failing, cfg, loc, thrower, exceptionClassName);
			return ExceptionGuard.unless(analysis, context, state, condition, pp);
		}

		/**
		 * Moves the executions raising the exceptions to the error channel,
		 * given the state of all the executions continuing normally.
		 *
		 * @param normal the state of the executions that do not raise any of
		 *                   the exceptions
		 *
		 * @return the state after the moves
		 *
		 * @throws SemanticException if the analysis fails
		 */
		public AnalysisState<A> moveFrom(
				AnalysisState<A> normal)
				throws SemanticException {
			AnalysisState<A> result = normal;
			for (Map.Entry<AnalysisState.Error, AnalysisState<A>> entry : raised.entrySet())
				result = analysis.moveExecutionToError(result, entry.getValue(), entry.getKey(),
						context, entry.getKey().getThrower());
			return result;
		}
	}

	/**
	 * Yields the statement of the program raising the errors raised by the
	 * given one: a call raises the errors of the statement it originates from
	 * (e.g. the call to a native function performed by a call of the program),
	 * and a call performed to implement the semantics of a statement (see
	 * {@link CallSites.Site}) raises the errors of that statement. Hence, the
	 * errors raised by a statement are the same ones, whatever implementation
	 * computes them (e.g. the {@code ZeroDivisionError} of {@code x // y}, for
	 * an {@code int} or a {@code float} {@code x}).
	 *
	 * @param thrower the statement raising the errors
	 *
	 * @return the statement of the program
	 */
	public static Statement programStatement(
			Statement thrower) {
		Statement result = thrower;
		while (true)
			if (result instanceof Call && ((Call) result).getSource() != null)
				result = ((Call) result).getSource();
			else if (result instanceof CallSites.Site)
				result = ((CallSites.Site) result).getCaller();
			else
				return result;
	}
}
