package it.unive.pylisa.libraries.conversions;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.bytes.CodecSemantics;
import it.unive.pylisa.symbolic.PyBytes;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.bytes.BytesUnary;
import it.unive.pylisa.symbolic.operators.bytes.Codec;

/**
 * Native implementation of
 * {@code bytes(source=None, encoding=None, errors=None)}:
 * <ul>
 * <li>{@code bytes()} is {@code b''};</li>
 * <li>{@code bytes(s, encoding, errors)} encodes the {@code str} {@code s} (see
 * {@link CodecSemantics}), and raises {@code TypeError} without an
 * {@code encoding};</li>
 * <li>{@code bytes(n)} is {@code n} zero bytes for an {@code int} {@code n},
 * and raises {@code ValueError} if {@code n} is negative;</li>
 * <li>{@code bytes(b)} is {@code b} for {@code bytes} {@code b};</li>
 * <li>an {@code encoding} or {@code errors} without a {@code str} source, and
 * any other source of a builtin type, raise {@code TypeError};</li>
 * <li>for other sources (e.g. lists of integers), the result is not computed
 * precisely, and {@code TypeError} or {@code ValueError} might be raised.</li>
 * </ul>
 * An explicit {@code None} is treated as an omitted argument.
 */
public class BytesBuiltin extends PyNative {

	protected BytesBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "bytes", params);
	}

	public static BytesBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression source = args[0], encoding = args[1], errors = args[2];
		boolean noCodec = encoding instanceof PyNoneConstant && errors instanceof PyNoneConstant;
		if (source instanceof PyNoneConstant)
			// bytes() is empty, bytes(encoding=...) raises TypeError
			return noCodec ? compute(analysis, state, new Constant(PyBytesType.INSTANCE, new PyBytes(new byte[0]), loc))
					: raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR);

		Satisfiability isStr = hasType(analysis, state, source, STR);
		Satisfiability isInt = hasType(analysis, state, source, INT);
		Satisfiability isBytes = hasType(analysis, state, source, BYTES);
		Satisfiability isObject = hasType(analysis, state, source, t -> !PyBinaryDispatch.isBuiltinValueType(t)
				&& !t.isNullType());

		AnalysisState<A> result = state.bottom();
		if (isStr != Satisfiability.NOT_SATISFIED)
			result = result.lub(encoding instanceof PyNoneConstant
					// a string argument without an encoding
					? raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR)
					: CodecSemantics.semantics(this, analysis, state, Codec.ENCODE, source, encoding, errors));
		if (isInt != Satisfiability.NOT_SATISFIED)
			result = result.lub(!noCodec ? raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR)
					: ExceptionGuard.guardedCompute(analysis, state,
							new BinaryExpression(BoolType.INSTANCE, source, new Constant(Int32Type.INSTANCE, 0, loc),
									ComparisonLt.INSTANCE, loc),
							LibrarySpecificationProvider.VALUE_ERROR,
							new UnaryExpression(PyBytesType.INSTANCE, source, BytesUnary.ZEROS, loc), getCFG(), loc,
							getOriginatingStatement(), this));
		if (isBytes != Satisfiability.NOT_SATISFIED)
			result = result.lub(!noCodec ? raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR)
					: compute(analysis, state, source));
		if (isObject != Satisfiability.NOT_SATISFIED)
			// e.g. a list of integers, whose elements are not known
			result = result.lub(unknown(analysis, state, PyBytesType.INSTANCE))
					.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR))
					.lub(raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
		if (isStr != Satisfiability.SATISFIED && isInt != Satisfiability.SATISFIED
				&& isBytes != Satisfiability.SATISFIED && isObject != Satisfiability.SATISFIED)
			// e.g. a float
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
