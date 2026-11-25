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
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
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
 * implementation, mirroring how JLiSA raises
 * {@code ArrayIndexOutOfBoundsException} in {@code JavaArrayAccess}: it
 * allocates an instance of the given
 * (built-in, field-less) exception class, wraps it in a {@link CFGThrow}, and
 * moves the resulting state to the error channel. The move is <i>total</i> (see
 * {@link Analysis#moveExecutionToError(AnalysisState, AnalysisState.Error, ScopeId, ProgramPoint)})
 * when all executions raise the exception, as in
 * {@link #raise(Analysis, ScopeId, AnalysisState, CFG, CodeLocation, Statement, String)},
 * whose returned state has an empty normal execution. When only some executions
 * raise it, the move must instead be <i>partial</i> (see
 * {@link Analysis#moveExecutionToError(AnalysisState, AnalysisState, AnalysisState.Error, ScopeId, ProgramPoint)}),
 * knowing all the executions that continue normally: {@link Raising} collects
 * the raising executions until these are known.<br>
 * <br>
 * The executions raising an exception follow a discipline, on which the
 * verification of the warnings relies: an exception is raised in all the
 * executions of a state only if it is exactly the executions that raise it
 * (e.g., narrowed to the ones where an operand has the failing types, or where
 * the failing condition holds); otherwise, it is raised under an unknown
 * condition, so that the verification never takes it for granted (see
 * {@link Raising#raise(Satisfiability, AnalysisState, CFG, CodeLocation, Statement, String)}).
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
	 * {@link #raiseWhere(SymbolicExpression, boolean, AnalysisState, Statement, CFG, CodeLocation, Statement, String)}
	 * and
	 * {@link #raiseOfTypes(AnalysisState, SymbolicExpression, Set, Set, boolean, CFG, CodeLocation, Statement, String)}),
	 * and {@link #moveFrom(AnalysisState)} performs the moves. If no execution
	 * continues normally, the moves fall back to <i>total</i> ones.<br>
	 * <br>
	 * An exception is raised with {@link Satisfiability#SATISFIED} only on a
	 * state made exactly of the executions raising it, and with
	 * {@link Satisfiability#UNKNOWN} (under an unknown condition) when the
	 * state might also hold executions that do not raise it (see
	 * {@link #certainOnlyIf(boolean)} for the states restricted to a
	 * combination of the operands, that might not be exact).
	 *
	 * @param <A> the kind of {@link AbstractLattice} of the states
	 * @param <D> the kind of {@link AbstractDomain} of the analysis
	 */
	public static final class Raising<A extends AbstractLattice<A>, D extends AbstractDomain<A>> {

		private final Analysis<A, D> analysis;

		private final ScopeId<?> context;

		// the state of the executions raising each error, not moved yet
		private final Map<AnalysisState.Error, AnalysisState<A>> raised;

		// whether the exceptions raised in all the executions of a state are
		// raised in all of them (see certainOnlyIf)
		private final boolean certain;

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
			this.raised = new LinkedHashMap<>();
			this.certain = true;
		}

		private Raising(
				Raising<A, D> other,
				boolean certain) {
			this.analysis = other.analysis;
			this.context = other.context;
			this.raised = other.raised;
			this.certain = other.certain && certain;
		}

		/**
		 * Yields a collector of the same exceptions as this one (moved together
		 * with them) that, unless the given condition holds, raises under an
		 * unknown condition also the exceptions that this would raise in all
		 * the executions of a state. This is used with states that might have
		 * more executions than the ones where the exceptions are raised (e.g.,
		 * the executions computing a combination of the operands of a
		 * statement, if they are not known exactly, see
		 * {@link it.unive.lisa.program.cfg.statement.NaryExpression#restrictsExactly}).
		 *
		 * @param exact whether the states are made exactly of the executions
		 *                  where the exceptions are raised
		 *
		 * @return the collector
		 */
		public Raising<A, D> certainOnlyIf(
				boolean exact) {
			return exact ? this : new Raising<>(this, false);
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

		/**
		 * As
		 * {@link #raiseOfTypes(AnalysisState, SymbolicExpression, Set, Set, boolean, CFG, CodeLocation, Statement, String)},
		 * for an operand that might be any of the given values (a value whose
		 * types are not known fails if {@link Untyped} is a failing type): in all the
		 * executions if all the types of all the values fail, and otherwise
		 * where each value has the failing types (under an unknown condition
		 * if the executions where each value is the operand are not known
		 * exactly). These are the executions where the given program point, if
		 * any, computed the value (see
		 * {@link Analysis#restrictTo(AnalysisState, ProgramPoint, SymbolicExpression)}).
		 *
		 * @param state              the state of the executions that might
		 *                               raise the exception
		 * @param values             the values of the operand
		 * @param computing          the program point computing the values,
		 *                               or {@code null} if it is not known
		 * @param failing            the types of the operand raising the
		 *                               exception
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
				Collection<SymbolicExpression> values,
				ProgramPoint computing,
				Set<Type> failing,
				boolean surely,
				CFG cfg,
				CodeLocation loc,
				Statement thrower,
				String exceptionClassName)
				throws SemanticException {
			if (failing.isEmpty() || values.isEmpty())
				return;
			Map<SymbolicExpression, Set<Type>> types = new LinkedHashMap<>();
			boolean all = true;
			for (SymbolicExpression v : values) {
				Set<Type> t = analysis.getRuntimeTypesOf(state, v, context, thrower);
				types.put(v, t);
				all &= !t.isEmpty() && t.stream().noneMatch(Type::isUntyped) && failing.containsAll(t);
			}
			if (values.size() == 1) {
				SymbolicExpression v = values.iterator().next();
				raiseOfTypes(state, v, failing, types.get(v), surely, cfg, loc, thrower, exceptionClassName);
				return;
			}
			if (all) {
				// every value fails
				raise(surely ? Satisfiability.SATISFIED : Satisfiability.UNKNOWN, state, cfg, loc, thrower,
						exceptionClassName);
				return;
			}
			for (Map.Entry<SymbolicExpression, Set<Type>> v : types.entrySet()) {
				// the failing types of this value (unknown ones, if unknown
				// types fail)
				Set<Type> ofValue = new java.util.HashSet<>();
				if (v.getValue().isEmpty() && failing.stream().anyMatch(Type::isUntyped))
					ofValue.add(Untyped.INSTANCE);
				for (Type t : v.getValue())
					if (failing.contains(t))
						ofValue.add(t);
				if (ofValue.isEmpty())
					continue;
				AnalysisState<A> computed = computing == null ? state
						: analysis.restrictTo(state, computing, v.getKey());
				// surely raised only if the executions computing the value
				// are known exactly
				boolean exact = computing != null && analysis.restrictsExactly(state, computing, v.getKey());
				if (!computed.getExecutionState().isBottom())
					raiseOfTypes(computed, v.getKey(), ofValue, v.getValue(), surely && exact, cfg, loc, thrower,
							exceptionClassName);
			}
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
			AnalysisState<A> raising = raises == Satisfiability.SATISFIED && certain ? state
					: ExceptionGuard.maybe(analysis, context, state, thrower);
			if (!raising.getExecutionState().isBottom())
				collect(raising, cfg, loc, thrower, exceptionClassName);
		}

		/**
		 * Raises the given exception exactly in the executions of the given
		 * state where the given condition holds, yielding the ones where it
		 * does not, that continue normally. The analysis decides whether the
		 * condition holds (see {@link Analysis#satisfies}): if it surely does,
		 * the exception is raised in all the executions (under an unknown
		 * condition, all of them continuing normally too, if the condition
		 * involves floating point numbers other than literals, that the
		 * analysis computes with less precision than python); if it surely does not, in none of them; otherwise, the state is restricted to the
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
			if (sat == Satisfiability.SATISFIED && floats(state, condition, pp)) {
				// the analysis computes floating point numbers with less
				// precision than python (e.g., 0.1 + 0.2 - 0.3 is 0 for it):
				// the condition might not hold, and the executions are not
				// restricted, as the analysis would decide them in the same
				// way
				raise(Satisfiability.UNKNOWN, state, cfg, loc, thrower, exceptionClassName);
				return state;
			}
			if (sat == Satisfiability.SATISFIED) {
				raise(raises, state, cfg, loc, thrower, exceptionClassName);
				return state.bottomExecution();
			}
			AnalysisState<A> failing = ExceptionGuard.when(analysis, context, state, condition, pp);
			if (!failing.getExecutionState().isBottom())
				raise(raises, failing, cfg, loc, thrower, exceptionClassName);
			return ExceptionGuard.unless(analysis, context, state, condition, pp);
		}

		// whether some operand of the condition might be a floating point
		// number computed by the analysis (literals keep their sign, and
		// whether they are zero, also with less precision)
		private boolean floats(
				AnalysisState<A> state,
				SymbolicExpression condition,
				ProgramPoint pp)
				throws SemanticException {
			if (condition instanceof UnaryExpression)
				return floats(state, ((UnaryExpression) condition).getExpression(), pp);
			if (condition instanceof BinaryExpression)
				return floats(state, ((BinaryExpression) condition).getLeft(), pp)
						|| floats(state, ((BinaryExpression) condition).getRight(), pp);
			if (condition instanceof TernaryExpression)
				return floats(state, ((TernaryExpression) condition).getLeft(), pp)
						|| floats(state, ((TernaryExpression) condition).getMiddle(), pp)
						|| floats(state, ((TernaryExpression) condition).getRight(), pp);
			if (condition instanceof Constant)
				return false;
			if (isFloat(condition.getStaticType()))
				return true;
			for (Type t : analysis.getRuntimeTypesOf(state, condition, context, pp))
				if (isFloat(t))
					return true;
			return false;
		}

		private static boolean isFloat(
				Type t) {
			return t.isNumericType() && !t.asNumericType().isIntegral();
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
