package it.unive.pylisa.libraries.natives;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingAdd;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingDiv;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.symbolic.value.operator.binary.StringContains;
import it.unive.lisa.symbolic.value.operator.binary.StringMatches;
import it.unive.lisa.symbolic.value.operator.binary.StringStartsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringSubstringToEnd;
import it.unive.lisa.symbolic.value.operator.unary.NumericFloor;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;

/**
 * Builds the symbolic expressions that library models evaluate, all placed at
 * the location of the modelled call. Strings are combined with LiSA's string
 * operators, so that every configured value domain can evaluate them.
 */
public final class Expressions {

	private final CodeLocation location;

	/**
	 * Builds the factory.
	 *
	 * @param location the location of the modelled call
	 */
	public Expressions(
			CodeLocation location) {
		this.location = location;
	}

	/**
	 * Yields the location of the modelled call.
	 *
	 * @return the location
	 */
	public CodeLocation location() {
		return location;
	}

	/**
	 * Yields a string constant.
	 *
	 * @param value the string
	 *
	 * @return the constant
	 */
	public SymbolicExpression string(
			String value) {
		return new Constant(StringType.INSTANCE, value, location);
	}

	/**
	 * Yields an integer constant.
	 *
	 * @param value the integer
	 *
	 * @return the constant
	 */
	public SymbolicExpression integer(
			int value) {
		return new Constant(Int32Type.INSTANCE, value, location);
	}

	/**
	 * Yields a boolean constant.
	 *
	 * @param value the boolean
	 *
	 * @return the constant
	 */
	public SymbolicExpression bool(
			boolean value) {
		return new Constant(BoolType.INSTANCE, value, location);
	}

	/**
	 * Yields the constant of a Python value given as a Java object: a string,
	 * a boolean, an integer or a float.
	 *
	 * @param value the value
	 *
	 * @return the constant
	 *
	 * @throws IllegalArgumentException if the value is of another kind
	 */
	public SymbolicExpression constant(
			Object value) {
		if (value instanceof String)
			return new Constant(StringType.INSTANCE, value, location);
		if (value instanceof Boolean)
			return new Constant(BoolType.INSTANCE, value, location);
		if (value instanceof Integer)
			return new Constant(Int32Type.INSTANCE, value, location);
		if (value instanceof Long)
			return new Constant(Int64Type.INSTANCE, value, location);
		if (value instanceof Double)
			return new Constant(Float64Type.INSTANCE, value, location);
		throw new IllegalArgumentException("Not a Python constant: " + value);
	}

	/**
	 * Yields a float constant.
	 *
	 * @param value the float
	 *
	 * @return the constant
	 */
	public SymbolicExpression real(
			double value) {
		return new Constant(Float64Type.INSTANCE, value, location);
	}

	/**
	 * Yields a value about which nothing is known.
	 *
	 * @param type the static type of the value
	 *
	 * @return the value
	 */
	public SymbolicExpression unknown(
			Type type) {
		return new PushAny(type, location);
	}

	/**
	 * Yields a value about which nothing is known, not even its type.
	 *
	 * @return the value
	 */
	public SymbolicExpression unknown() {
		return unknown(Untyped.INSTANCE);
	}

	/**
	 * Yields the sum of two numbers, with Python semantics.
	 *
	 * @param left  the first addend
	 * @param right the second addend
	 *
	 * @return the sum
	 */
	public SymbolicExpression plus(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(Untyped.INSTANCE, NumericNonOverflowingAdd.INSTANCE, left, right);
	}

	/**
	 * Yields the quotient of two numbers, as Python's true division
	 * computes it when the dividend is a float.
	 *
	 * @param left  the dividend
	 * @param right the divisor
	 *
	 * @return the quotient
	 */
	public SymbolicExpression divide(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(Untyped.INSTANCE, NumericNonOverflowingDiv.INSTANCE, left, right);
	}

	/**
	 * Yields the product of two numbers, with Python semantics.
	 *
	 * @param left  the first factor
	 * @param right the second factor
	 *
	 * @return the product
	 */
	public SymbolicExpression times(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(Untyped.INSTANCE, NumericNonOverflowingMul.INSTANCE, left, right);
	}

	/**
	 * Yields the largest integer not greater than a number, as
	 * {@code math.floor} computes it.
	 *
	 * @param value the number
	 *
	 * @return the integer
	 */
	public SymbolicExpression floor(
			SymbolicExpression value) {
		return new UnaryExpression(Int64Type.INSTANCE, value, NumericFloor.INSTANCE, location);
	}

	/**
	 * Yields the condition {@code left < right}, with Python semantics.
	 *
	 * @param left  the first operand
	 * @param right the second operand
	 *
	 * @return the condition
	 */
	public SymbolicExpression lessThan(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(BoolType.INSTANCE, PyComparisonLt.INSTANCE, left, right);
	}

	/**
	 * Yields the condition {@code left <= right}, with Python semantics.
	 *
	 * @param left  the first operand
	 * @param right the second operand
	 *
	 * @return the condition
	 */
	public SymbolicExpression lessOrEqual(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(BoolType.INSTANCE, PyComparisonLe.INSTANCE, left, right);
	}

	/**
	 * Yields Python's {@code None}.
	 *
	 * @return the constant
	 */
	public SymbolicExpression none() {
		return new PyNoneConstant(location);
	}

	/**
	 * Yields the concatenation of strings, from left to right.
	 *
	 * @param first the first string
	 * @param rest  the other strings
	 *
	 * @return the concatenation
	 */
	public SymbolicExpression concat(
			SymbolicExpression first,
			SymbolicExpression... rest) {
		SymbolicExpression result = first;
		for (SymbolicExpression next : rest)
			result = binary(StringType.INSTANCE, StringConcat.INSTANCE, result, next);
		return result;
	}

	/**
	 * Yields the suffix of a string that starts at the given index.
	 *
	 * @param string the string
	 * @param begin  the index of the first character of the suffix
	 *
	 * @return the suffix
	 */
	public SymbolicExpression suffix(
			SymbolicExpression string,
			int begin) {
		return binary(StringType.INSTANCE, StringSubstringToEnd.INSTANCE, string, integer(begin));
	}

	/**
	 * Yields the condition {@code left == right}, with Python semantics.
	 *
	 * @param left  the first operand
	 * @param right the second operand
	 *
	 * @return the condition
	 */
	public SymbolicExpression equal(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(BoolType.INSTANCE, ComparisonEq.INSTANCE, left, right);
	}

	/**
	 * Yields the condition {@code value is None}.
	 *
	 * @param value the value
	 *
	 * @return the condition
	 */
	public SymbolicExpression isNone(
			SymbolicExpression value) {
		return equal(value, none());
	}

	/**
	 * Yields the condition that holds when either of two conditions does.
	 *
	 * @param left  the first condition
	 * @param right the second condition
	 *
	 * @return the disjunction
	 */
	public SymbolicExpression or(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(BoolType.INSTANCE, LogicalOr.INSTANCE, left, right);
	}

	/**
	 * Yields the condition that holds when both of two conditions do.
	 *
	 * @param left  the first condition
	 * @param right the second condition
	 *
	 * @return the conjunction
	 */
	public SymbolicExpression and(
			SymbolicExpression left,
			SymbolicExpression right) {
		return binary(BoolType.INSTANCE, LogicalAnd.INSTANCE, left, right);
	}

	/**
	 * Yields the condition that a string starts with a prefix.
	 *
	 * @param string the string
	 * @param prefix the prefix
	 *
	 * @return the condition
	 */
	public SymbolicExpression startsWith(
			SymbolicExpression string,
			String prefix) {
		return binary(BoolType.INSTANCE, StringStartsWith.INSTANCE, string, string(prefix));
	}

	/**
	 * Yields the condition that a string contains another one.
	 *
	 * @param string the string
	 * @param part   the contained string
	 *
	 * @return the condition
	 */
	public SymbolicExpression contains(
			SymbolicExpression string,
			String part) {
		return binary(BoolType.INSTANCE, StringContains.INSTANCE, string, string(part));
	}

	/**
	 * Yields the condition that a whole string matches a regular expression.
	 *
	 * @param string  the string
	 * @param pattern the regular expression
	 *
	 * @return the condition
	 */
	public SymbolicExpression matches(
			SymbolicExpression string,
			String pattern) {
		return binary(BoolType.INSTANCE, StringMatches.INSTANCE, string, string(pattern));
	}

	private SymbolicExpression binary(
			it.unive.lisa.type.Type type,
			BinaryOperator operator,
			SymbolicExpression left,
			SymbolicExpression right) {
		return new BinaryExpression(type, left, right, operator, location);
	}
}
