package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.Codec;
import it.unive.pylisa.symbolic.operators.bytes.CodecRaises;

/**
 * Shared semantics of {@code str.encode(encoding=None, errors=None)},
 * {@code bytes.decode(encoding=None, errors=None)} and {@code bytes(s,
 * encoding, errors)}: {@code encoding} and {@code errors} must be {@code str}s
 * (or {@code None}, for {@code "utf-8"} and {@code "strict"}), otherwise
 * {@code TypeError} is raised. Encoding or decoding fails with
 * {@code UnicodeEncodeError} or {@code UnicodeDecodeError}, and an unknown
 * encoding or error handler raises {@code LookupError}: the domains decide
 * which of these can happen.
 */
public final class CodecSemantics {

	private CodecSemantics() {
	}

	/**
	 * Encodes or decodes {@code value}.
	 *
	 * @param n        the native performing the operation
	 * @param analysis the analysis
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
			AnalysisState<A> state,
			Codec codec,
			SymbolicExpression value,
			SymbolicExpression encoding,
			SymbolicExpression errors)
			throws SemanticException {
		CodeLocation loc = n.getLocation();
		Satisfiability typed = n.hasType(analysis, state, encoding, PyNative.STR.or(PyNative.NONE))
				.and(n.hasType(analysis, state, errors, PyNative.STR.or(PyNative.NONE)));

		String unicodeError = codec.isEncode() ? LibrarySpecificationProvider.UNICODE_ENCODE_ERROR
				: LibrarySpecificationProvider.UNICODE_DECODE_ERROR;
		Satisfiability failsUnicode = analysis.satisfies(state, new TernaryExpression(BoolType.INSTANCE, value,
				encoding, errors, new CodecRaises(codec, unicodeError), loc), n);
		Satisfiability failsLookup = analysis.satisfies(state, new TernaryExpression(BoolType.INSTANCE, value,
				encoding, errors, new CodecRaises(codec, LibrarySpecificationProvider.LOOKUP_ERROR), loc), n);

		AnalysisState<A> result = state.bottom();
		if (failsUnicode != Satisfiability.SATISFIED && failsLookup != Satisfiability.SATISFIED)
			result = result.lub(n.compute(analysis, state,
					new TernaryExpression(codec.isEncode() ? PyBytesType.INSTANCE : StringType.INSTANCE, value,
							encoding, errors, codec, loc)));
		if (failsUnicode != Satisfiability.NOT_SATISFIED)
			result = result.lub(n.raise(analysis, state, unicodeError));
		if (failsLookup != Satisfiability.NOT_SATISFIED)
			result = result.lub(n.raise(analysis, state, LibrarySpecificationProvider.LOOKUP_ERROR));
		return n.typeChecked(analysis, state, typed, result);
	}
}
