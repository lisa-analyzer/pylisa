package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.strings.StrSearch;
import java.util.function.Predicate;

/**
 * Shared semantics of {@code find}, {@code rfind}, {@code index},
 * {@code rindex}, {@code count}, {@code startswith} and {@code endswith} of
 * {@code str} and {@code bytes}, that all take
 * {@code (sub, start=None, end=None)}:
 * <ul>
 * <li>{@code sub} must have the same type of the receiver, or be an {@code int}
 * between 0 and 255 for {@code bytes} (otherwise {@code ValueError} is raised),
 * except for {@code startswith} and {@code endswith}, that also accept a tuple
 * (whose result is not computed precisely); {@code start} and {@code end} must
 * be {@code int}s or {@code None}, otherwise {@code TypeError} is raised;</li>
 * <li>{@code index} and {@code rindex} raise {@code ValueError} when the
 * substring is not found;</li>
 * <li>without bounds, a search in a {@code str} is expressed with the SDK's
 * string operators (e.g. {@code StringIndexOf}), so that any string domain can
 * evaluate it; otherwise, it is a {@link StrSearch} on the slice with the
 * bounds.</li>
 * </ul>
 */
final class StrSearchSemantics {

	private StrSearchSemantics() {
	}

	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			PyNative n,
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args,
			StrSearch search,
			BinaryOperator unbounded,
			boolean raiseIfMissing)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		for (boolean bytes : n.textModes(analysis, state, args[0]))
			result = result.lub(semantics(n, analysis, state, args, search, unbounded, raiseIfMissing, bytes));
		return result;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			PyNative n,
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args,
			StrSearch search,
			BinaryOperator unbounded,
			boolean raiseIfMissing,
			boolean bytes)
			throws SemanticException {
		CodeLocation loc = n.getLocation();
		SymbolicExpression s = args[0], sub = args[1], start = args[2], end = args[3];
		boolean prefix = search.getKind() == StrSearch.Kind.STARTSWITH || search.getKind() == StrSearch.Kind.ENDSWITH;
		Type type = prefix ? BoolType.INSTANCE : Int32Type.INSTANCE;

		// str searches str, bytes search bytes or (except for prefixes) a
		// single byte
		Satisfiability isText = n.hasType(analysis, state, sub, bytes ? PyNative.BYTES : PyNative.STR);
		Satisfiability isByte = bytes && !prefix ? n.hasType(analysis, state, sub, PyNative.INT)
				: Satisfiability.NOT_SATISFIED;
		Satisfiability isTuple = prefix ? n.hasType(analysis, state, sub, StrSearchSemantics::isTuple)
				: Satisfiability.NOT_SATISFIED;
		Satisfiability bounds = n.hasType(analysis, state, start, PyNative.INT.or(PyNative.NONE))
				.and(n.hasType(analysis, state, end, PyNative.INT.or(PyNative.NONE)));

		SymbolicExpression value;
		if (!bytes && unbounded != null && start instanceof PyNoneConstant && end instanceof PyNoneConstant)
			value = new BinaryExpression(type, s, sub, unbounded, loc);
		else {
			Type sliceType = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
			TernaryExpression slice = new TernaryExpression(sliceType, start, end, new PyNoneConstant(loc),
					SliceCreation.INSTANCE, loc);
			value = new TernaryExpression(type, s, sub, slice, search, loc);
		}

		AnalysisState<A> result = state.bottom();
		if (isText != Satisfiability.NOT_SATISFIED)
			result = result.lub(n.typeChecked(analysis, state, bounds,
					computed(n, analysis, state, value, raiseIfMissing)));

		if (isByte != Satisfiability.NOT_SATISFIED) {
			// a single byte must be between 0 and 255
			BinaryExpression outOfRange = new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, sub, new Constant(Int32Type.INSTANCE, 0, loc),
							ComparisonLt.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, sub, new Constant(Int32Type.INSTANCE, 255, loc),
							ComparisonGt.INSTANCE, loc),
					LogicalOr.INSTANCE, loc);
			Satisfiability out = analysis.satisfies(state, outOfRange, n);
			AnalysisState<A> byteResult = state.bottom();
			if (out != Satisfiability.SATISFIED)
				byteResult = byteResult.lub(computed(n, analysis, state, value, raiseIfMissing));
			if (out != Satisfiability.NOT_SATISFIED)
				byteResult = byteResult.lub(n.raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
			result = result.lub(n.typeChecked(analysis, state, bounds, byteResult));
		}

		if (isTuple != Satisfiability.NOT_SATISFIED)
			// the elements of the tuple are not known: they might also not have
			// the right type
			result = result.lub(n.unknown(analysis, state, BoolType.INSTANCE))
					.lub(n.raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));

		if (isText != Satisfiability.SATISFIED && isByte != Satisfiability.SATISFIED
				&& isTuple != Satisfiability.SATISFIED)
			result = result.lub(n.raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	// the search, raising ValueError if needed when the substring is missing
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> computed(
			PyNative n,
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			boolean raiseIfMissing)
			throws SemanticException {
		if (!raiseIfMissing)
			return n.compute(analysis, state, value);
		CodeLocation loc = n.getLocation();
		BinaryExpression missing = new BinaryExpression(BoolType.INSTANCE, value,
				new Constant(Int32Type.INSTANCE, -1, loc), ComparisonEq.INSTANCE, loc);
		return ExceptionGuard.guardedCompute(analysis, state, missing, LibrarySpecificationProvider.VALUE_ERROR,
				value, n.getCFG(), loc, n.getOriginatingStatement(), n);
	}

	private static boolean isTuple(
			Type t) {
		Predicate<Type> tuple = PyClassType.lookup(LibrarySpecificationProvider.TUPLE)::equals;
		return t.isPointerType() && tuple.test(t.asPointerType().getInnerType());
	}
}
