package it.unive.pylisa.cfg.expression;

import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
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
		Map<Statement, Map<Key, UnresolvedCall>> calls = CALLS.computeIfAbsent(interprocedural,
				i -> new IdentityHashMap<>());
		synchronized (calls) {
			return calls.computeIfAbsent(caller, c -> new HashMap<>()).computeIfAbsent(
					new Key(kind, qualifier, name, List.of(args)),
					k -> new Site(caller, kind, qualifier, name, args));
		}
	}

	/**
	 * A call performed by a statement of the program to implement its semantics
	 * (e.g. {@code __floordiv__} for {@code x // y}): the errors raised by the
	 * call are raised by that statement (see
	 * {@link it.unive.pylisa.libraries.PyExceptions}).
	 */
	public static final class Site extends UnresolvedCall {

		private final Statement caller;

		private Site(
				Statement caller,
				CallType kind,
				String qualifier,
				String name,
				Expression... args) {
			super(caller.getCFG(), caller.getLocation(), kind, qualifier, name, LeftToRightEvaluation.INSTANCE, args);
			this.caller = caller;
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
