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
 * Shared semantics of {@code str.find}, {@code rfind}, {@code index},
 * {@code rindex}, {@code count}, {@code startswith} and {@code endswith}, that
 * all take {@code (sub, start=None, end=None)}:
 * <ul>
 * <li>{@code sub} must be a {@code str} ({@code startswith} and
 * {@code endswith} also accept a tuple of {@code str}s, whose result is not
 * computed precisely), and {@code start} and {@code end} must be {@code int}s
 * or {@code None}, otherwise {@code TypeError} is raised;</li>
 * <li>{@code index} and {@code rindex} raise {@code ValueError} when the
 * substring is not found;</li>
 * <li>without bounds, the search is expressed with the SDK's string operators
 * (e.g. {@code StringIndexOf}), so that any string domain can evaluate it;
 * otherwise, it is a {@link StrSearch} on the slice with the bounds.</li>
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
		CodeLocation loc = n.getLocation();
		SymbolicExpression s = args[0], sub = args[1], start = args[2], end = args[3];
		boolean prefix = search.getKind() == StrSearch.Kind.STARTSWITH || search.getKind() == StrSearch.Kind.ENDSWITH;
		Type type = prefix ? BoolType.INSTANCE : Int32Type.INSTANCE;

		Satisfiability isStr = n.hasType(analysis, state, sub, PyNative.STR);
		Satisfiability isTuple = prefix ? n.hasType(analysis, state, sub, StrSearchSemantics::isTuple)
				: Satisfiability.NOT_SATISFIED;
		Satisfiability bounds = n.hasType(analysis, state, start, PyNative.INT.or(PyNative.NONE))
				.and(n.hasType(analysis, state, end, PyNative.INT.or(PyNative.NONE)));

		AnalysisState<A> result = state.bottom();
		if (isStr != Satisfiability.NOT_SATISFIED) {
			SymbolicExpression value;
			if (unbounded != null && start instanceof PyNoneConstant && end instanceof PyNoneConstant)
				value = new BinaryExpression(type, s, sub, unbounded, loc);
			else {
				Type sliceType = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
				TernaryExpression slice = new TernaryExpression(sliceType, start, end, new PyNoneConstant(loc),
						SliceCreation.INSTANCE, loc);
				value = new TernaryExpression(type, s, sub, slice, search, loc);
			}

			AnalysisState<A> computed;
			if (raiseIfMissing) {
				BinaryExpression missing = new BinaryExpression(BoolType.INSTANCE, value,
						new Constant(Int32Type.INSTANCE, -1, loc), ComparisonEq.INSTANCE, loc);
				computed = ExceptionGuard.guardedCompute(analysis, state, missing,
						LibrarySpecificationProvider.VALUE_ERROR, value, n.getCFG(), loc, n.getOriginatingStatement(),
						n);
			} else
				computed = n.compute(analysis, state, value);
			result = result.lub(n.typeChecked(analysis, state, bounds, computed));
		}

		if (isTuple != Satisfiability.NOT_SATISFIED)
			// the elements of the tuple are not known: they might also not be
			// strings
			result = result.lub(n.unknown(analysis, state, BoolType.INSTANCE))
					.lub(n.raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));

		if (isStr != Satisfiability.SATISFIED && isTuple != Satisfiability.SATISFIED)
			result = result.lub(n.raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	private static boolean isTuple(
			Type t) {
		Predicate<Type> tuple = PyClassType.lookup(LibrarySpecificationProvider.TUPLE)::equals;
		return t.isPointerType() && tuple.test(t.asPointerType().getInnerType());
	}
}
