package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.Codec;
import it.unive.pylisa.symbolic.operators.bytes.CodecRaises;

/**
 * Shared semantics of {@code str.encode([encoding[, errors]])},
 * {@code bytes.decode([encoding[, errors]])} and {@code bytes(s,
 * encoding, errors)}: {@code encoding} and {@code errors} must be {@code str}s
 * (when they are passed: they are {@code "utf-8"} and {@code "strict"}
 * otherwise), so an explicit {@code None} raises {@code TypeError}. Encoding or
 * decoding fails with {@code UnicodeEncodeError} or {@code UnicodeDecodeError},
 * and an unknown encoding or error handler raises {@code LookupError}: the
 * domains decide which of these can happen.
 */
public final class CodecSemantics {

	private CodecSemantics() {
	}

	/**
	 * Encodes or decodes {@code value}.
	 *
	 * @param n        the native performing the operation
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param codec    the operation
	 * @param value    the value to encode or decode
	 * @param encoding the encoding
	 * @param errors   the error handler
	 *
	 * @return the state after the operation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			PyNative n,
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			Codec codec,
			SymbolicExpression value,
			SymbolicExpression encoding,
			SymbolicExpression errors)
			throws SemanticException {
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		return raising.moveFrom(semantics(n, analysis, context, state, codec, value, encoding, errors, raising));
	}

	/**
	 * Encodes or decodes {@code value}, as
	 * {@link #semantics(PyNative, Analysis, ScopeId, AnalysisState, Codec, SymbolicExpression, SymbolicExpression, SymbolicExpression)},
	 * but the raised exceptions are collected in {@code raising} instead of
	 * being moved to the error channel.
	 *
	 * @param n        the native performing the operation
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param codec    the operation
	 * @param value    the value to encode or decode
	 * @param encoding the encoding
	 * @param errors   the error handler
	 * @param raising  where the raised exceptions are collected
	 *
	 * @return the state after the operation, where the executions raising
	 *             exceptions have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			PyNative n,
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			Codec codec,
			SymbolicExpression value,
			SymbolicExpression encoding,
			SymbolicExpression errors,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		CodeLocation loc = n.getLocation();
		Satisfiability typed = (PyNative.missing(encoding) ? Satisfiability.SATISFIED
				: n.hasType(analysis, context, state, encoding, PyNative.STR))
						.and(PyNative.missing(errors) ? Satisfiability.SATISFIED
								: n.hasType(analysis, context, state, errors, PyNative.STR));
		// the operators model a missing argument as None
		encoding = PyNative.orNone(encoding);
		errors = PyNative.orNone(errors);

		String unicodeError = codec.isEncode() ? LibrarySpecificationProvider.UNICODE_ENCODE_ERROR
				: LibrarySpecificationProvider.UNICODE_DECODE_ERROR;
		TernaryExpression unicodeFailure = new TernaryExpression(BoolType.INSTANCE, value,
				encoding, errors, new CodecRaises(codec, unicodeError), loc);
		TernaryExpression lookupFailure = new TernaryExpression(BoolType.INSTANCE, value,
				encoding, errors, new CodecRaises(codec, LibrarySpecificationProvider.LOOKUP_ERROR), loc);
		Satisfiability failsUnicode = analysis.satisfies(state, unicodeFailure, context, n);
		Satisfiability failsLookup = analysis.satisfies(state, lookupFailure, context, n);

		AnalysisState<A> result = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			// the codec is applied only to well-typed arguments, where it does
			// not fail, and each exception is raised exactly where its
			// condition holds
			if (failsUnicode != Satisfiability.SATISFIED && failsLookup != Satisfiability.SATISFIED) {
				AnalysisState<A> succeeding = state;
				if (failsUnicode == Satisfiability.UNKNOWN)
					succeeding = ExceptionGuard.unless(analysis, context, succeeding, unicodeFailure, n);
				if (failsLookup == Satisfiability.UNKNOWN)
					succeeding = ExceptionGuard.unless(analysis, context, succeeding, lookupFailure, n);
				if (!succeeding.getExecutionState().isBottom())
					result = result.lub(n.compute(analysis, context, succeeding,
							new TernaryExpression(codec.isEncode() ? PyBytesType.INSTANCE : StringType.INSTANCE,
									value, encoding, errors, codec, loc)));
			}
			raiseWhere(n, analysis, context, state, failsUnicode, unicodeFailure, unicodeError, raising);
			raiseWhere(n, analysis, context, state, failsLookup, lookupFailure,
					LibrarySpecificationProvider.LOOKUP_ERROR, raising);
		}
		return n.typeChecked(state, typed, result, raising);
	}

	// raises the exception where its condition holds
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> void raiseWhere(
			PyNative n,
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			Satisfiability fails,
			SymbolicExpression failure,
			String exception,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		if (fails == Satisfiability.NOT_SATISFIED || fails == Satisfiability.BOTTOM)
			return;
		AnalysisState<A> failing = fails == Satisfiability.SATISFIED ? state
				: ExceptionGuard.when(analysis, context, state, failure, n);
		if (!failing.getExecutionState().isBottom())
			n.raise(raising, Satisfiability.SATISFIED, failing, exception);
	}
}
