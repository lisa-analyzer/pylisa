package it.unive.pylisa.cfg.expression;

import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * The calls that statements resolve during the analysis (e.g. the
 * {@code __eq__} called by {@code ==}). The call graph caches resolutions by
 * the identity of the call, keeping every call it is asked to resolve: a call
 * built anew at each evaluation is never found in the cache, and it is kept
 * forever. With operands of unknown types, a statement resolves a call for each
 * pair of types at each evaluation, exhausting the memory. Here, the same call
 * is built once for each statement, for each analysis.
 */
public final class CallSites {

	private record Key(
			CodeLocation location,
			CallType kind,
			String qualifier,
			String name,
			List<Expression> args) {
	}

	// the analyses are compared by identity, and they are not reachable from
	// the calls: the calls of an analysis are dropped with it
	private static final Map<InterproceduralAnalysis<?, ?>,
			Map<Statement, Map<Key, UnresolvedCall>>> CALLS = Collections
					.synchronizedMap(new WeakHashMap<>());

	private CallSites() {
	}

	/**
	 * Clears the caches of this class, so that sequential analyses in the same
	 * JVM do not interfere with each other.
	 */
	public static void clearAll() {
		CALLS.clear();
	}

	/**
	 * Yields the call {@code qualifier.name(args)} performed by the given
	 * statement, always the same one for the same analysis.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param caller          the statement performing the call
	 * @param kind            the kind of the call
	 * @param qualifier       the qualifier of the call, or {@code null}
	 * @param name            the name of the called method
	 * @param args            the arguments of the call
	 *
	 * @return the call
	 */
	public static UnresolvedCall get(
			InterproceduralAnalysis<?, ?> interprocedural,
			Statement caller,
			CallType kind,
			String qualifier,
			String name,
			Expression... args) {
		return get(interprocedural, caller, caller.getLocation(), kind, qualifier, name, args);
	}

	/**
	 * Yields the call with the given location, made by {@code caller}: the
	 * location distinguishes it from the other calls at the location of
	 * {@code caller}, whose values are stored in variables named after it (e.g.
	 * the call of a bound method returned by another call, as in
	 * {@code o.f()()}, see {@link BoundMethods}).
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param caller          the statement making the call
	 * @param location        the location of the call
	 * @param kind            the kind of the call
	 * @param qualifier       the qualifier of the call
	 * @param name            the name of the called function
	 * @param args            the arguments of the call
	 *
	 * @return the call
	 */
	public static UnresolvedCall get(
			InterproceduralAnalysis<?, ?> interprocedural,
			Statement caller,
			CodeLocation location,
			CallType kind,
			String qualifier,
			String name,
			Expression... args) {
		Map<Statement, Map<Key, UnresolvedCall>> calls = CALLS.computeIfAbsent(interprocedural,
				i -> new IdentityHashMap<>());
		synchronized (calls) {
			return calls.computeIfAbsent(caller, c -> new HashMap<>()).computeIfAbsent(
					new Key(location, kind, qualifier, name, List.of(args)),
					k -> new Site(caller, location, kind, qualifier, name, args));
		}
	}

	/**
	 * A call performed by a statement of the program to implement its semantics
	 * (e.g. {@code __floordiv__} for {@code x // y}): the errors raised by the
	 * call are raised by that statement (see
	 * {@link it.unive.pylisa.libraries.PyExceptions}).
	 */
	/**
	 * Makes the given call, created by {@code caller} (and not part of the
	 * cfg), a sub-expression of {@code caller}: the errors raised by the
	 * targets of the call, that are attributed to the call, are then raised by
	 * the statement containing {@code caller} (e.g. for the except blocks
	 * protecting it, see {@code ProtectedBlock}).
	 *
	 * @param <C>    the kind of call
	 * @param call   the call
	 * @param caller the statement creating the call
	 *
	 * @return the call
	 */
	public static <C extends Call> C within(
			C call,
			Statement caller) {
		call.setParentStatement(caller);
		return call;
	}

	public static final class Site extends UnresolvedCall {

		private final Statement caller;

		private Site(
				Statement caller,
				CodeLocation location,
				CallType kind,
				String qualifier,
				String name,
				Expression... args) {
			super(caller.getCFG(), location, kind, qualifier, name, LeftToRightEvaluation.INSTANCE, args);
			this.caller = caller;
			// the errors of the call are raised by the caller (e.g. for the
			// except blocks protecting it)
			setParentStatement(caller);
		}

		/**
		 * Yields the statement performing the call.
		 *
		 * @return the statement
		 */
		public Statement getCaller() {
			return caller;
		}
	}
}
