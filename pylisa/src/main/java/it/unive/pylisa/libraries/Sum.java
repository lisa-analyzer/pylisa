package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingAdd;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Native implementation of Python's {@code sum(iterable, start=0)}, that adds
 * the elements to {@code start}, from left to right. When {@code start} and the
 * elements are numbers, the elements of a list or tuple of known length are
 * added one by one (as {@code int.__add__} and {@code float.__add__} do);
 * otherwise, the result is any number of the type of the sum. Adding a value of
 * a builtin type that is not a number (e.g. a {@code str}) raises
 * {@code TypeError}, as adding an object whose class does not define
 * {@code __radd__} does. The other additions of objects (or to a {@code start}
 * that is not a number, as in {@code sum(lists, [])}) are not executed: their
 * results, and the errors they might raise, are not known.
 */
public class Sum extends PyNative {

	protected Sum(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "sum", params);
	}

	public static Sum build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Sum(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression iterable = args[0], start = args[1];
		Set<Type> starts = analysis.getRuntimeTypesOf(state, start, this);
		AnalysisState<A> result = state.bottomExecution();

		List<SymbolicExpression> exact = ListModel.exactElements(analysis, state, iterable, this);
		Set<Type> elements = new HashSet<>();
		boolean empty, nonEmpty;
		if (exact != null) {
			// whether start and the elements so far are numbers
			boolean numbers = !starts.isEmpty() && starts.stream().allMatch(Sum::number);
			for (SymbolicExpression e : exact) {
				Set<Type> types = analysis.getRuntimeTypesOf(state, e, this);
				if (numbers && !types.isEmpty() && types.stream().allMatch(Sum::notAddable))
					// e.g. sum([1, "a"])
					return raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR);
				numbers &= !types.isEmpty() && types.stream().allMatch(Sum::number);
				elements.addAll(types);
			}
			empty = exact.isEmpty();
			nonEmpty = !empty;
			if (!starts.isEmpty() && starts.stream().allMatch(Sum::exactNumber) && elements.stream()
					.allMatch(Sum::exactNumber))
				// the elements are added one by one
				return compute(analysis, state, add(start, exact, starts, elements));
		} else {
			AnalysisState<A> any = Iteration.elements(analysis, state, getSubExpressions()[0], iterable, st,
					getMetaVariables());
			// e.g. the TypeError raised by iterating an int
			result = any.bottomExecution();
			if (any.getExecutionState().isBottom())
				return result;
			for (SymbolicExpression e : any.getExecutionExpressions())
				elements.addAll(analysis.getRuntimeTypesOf(any, e, this));
			empty = any.getExecutionExpressions().isEmpty();
			nonEmpty = nonEmpty(analysis, state, iterable);
		}

		if (starts.isEmpty() || !starts.stream().allMatch(Sum::number)) {
			// e.g. sum(lists, []): the additions are not executed
			result = result.lub(unknown(analysis, state, Untyped.INSTANCE));
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
			if (starts.isEmpty() || starts.stream().anyMatch(Sum::object) || elements.stream().anyMatch(Sum::object))
				result = result.lub(raise(analysis, state, LibrarySpecificationProvider.EXCEPTION));
			return result;
		}

		if (!nonEmpty)
			// no elements: the result is start
			result = result.lub(compute(analysis, state, start));
		if (empty)
			return result;
		Set<Type> numbers = new HashSet<>();
		for (Type t : elements)
			if (number(t))
				numbers.add(t);
		if (!numbers.isEmpty()) {
			// at least one number is added to start: the sum is a float if
			// start or all the elements are floats, and might be one if any
			// of them is
			boolean floatStart = starts.stream().allMatch(Sum::isFloat);
			boolean floats = numbers.stream().allMatch(Sum::isFloat);
			boolean someFloat = starts.stream().anyMatch(Sum::isFloat) || numbers.stream().anyMatch(Sum::isFloat);
			if (floatStart || floats || someFloat)
				result = result.lub(unknown(analysis, state, Float32Type.INSTANCE));
			if (!floatStart && !floats)
				result = result.lub(unknown(analysis, state, Int32Type.INSTANCE));
		}
		if (elements.stream().anyMatch(Sum::notAddable))
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		if (elements.isEmpty() || elements.stream().anyMatch(Sum::object)) {
			// e.g. an object whose class defines __radd__
			result = result.lub(unknown(analysis, state, Untyped.INSTANCE));
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.EXCEPTION));
		}
		return result;
	}

	// start + e0 + e1 + ...
	private SymbolicExpression add(
			SymbolicExpression start,
			List<SymbolicExpression> elements,
			Set<Type> starts,
			Set<Type> types) {
		Type type = starts.stream().anyMatch(Sum::isFloat) || types.stream().anyMatch(Sum::isFloat)
				? Float32Type.INSTANCE
				: Int32Type.INSTANCE;
		SymbolicExpression sum = start;
		for (SymbolicExpression e : elements)
			sum = new BinaryExpression(type, sum, e, NumericNonOverflowingAdd.INSTANCE, getLocation());
		return sum;
	}

	// whether the iterable is a list or tuple that surely has some elements
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean nonEmpty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterable)
			throws SemanticException {
		if (!analysis.getRuntimeTypesOf(state, iterable, this).stream()
				.allMatch(t -> ListModel.isList(t) || Unpacking.isTuple(t)))
			return false;
		ListModel.Shape shape = ListModel.shape(analysis, state, iterable, this);
		return shape.prefix() > 0 || (shape.length() != null && shape.length() > 0);
	}

	// whether the values of type t are numbers (a bool is an int)
	private static boolean number(
			Type t) {
		return t.isNumericType() || t.isBooleanType();
	}

	// whether the values of type t are ints or floats, whose additions are
	// computed as values
	private static boolean exactNumber(
			Type t) {
		return t.isNumericType();
	}

	private static boolean isFloat(
			Type t) {
		return t.isNumericType() && !t.asNumericType().isIntegral();
	}

	// whether the values of type t cannot be added to a number: the values of
	// builtin types that are not numbers (e.g. a str, a list or None), and the
	// objects of classes whose methods are all known, that do not define
	// __radd__
	private static boolean notAddable(
			Type t) {
		if (number(t))
			return false;
		if (PyBinaryDispatch.isBuiltinValueType(t) || t.isCodeMemberType() || ListModel.isList(t)
				|| Unpacking.isTuple(t) || DictModel.isDict(t))
			return true;
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType() || !PyBinaryDispatch.fullyKnown(t)
				|| !(t.asPointerType().getInnerType().asUnitType().getUnit() instanceof CompilationUnit))
			return false;
		CompilationUnit unit = (CompilationUnit) t.asPointerType().getInnerType().asUnitType().getUnit();
		return unit.getInstanceCodeMembersByName("__radd__", true).isEmpty();
	}

	// whether the values of type t are objects, whose additions are not known
	private static boolean object(
			Type t) {
		return !number(t) && !notAddable(t);
	}
}
