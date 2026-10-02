package it.unive.pylisa.analysis.constants;

import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.symbolic.value.operator.binary.StringContains;
import it.unive.lisa.symbolic.value.operator.binary.StringEndsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringEquals;
import it.unive.lisa.symbolic.value.operator.binary.StringMatches;
import it.unive.lisa.symbolic.value.operator.binary.StringStartsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringSubstringToEnd;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Optional;
import java.util.regex.PatternSyntaxException;

/**
 * The concrete semantics of the binary operators that {@link ConstantPropagation}
 * evaluates exactly when both operands are known constants: string
 * concatenation, suffix extraction, the string predicates, Python's
 * {@code ==} / {@code !=}, and the orderings of numbers.
 * <p>
 * Every method yields an empty result when the concrete outcome cannot be
 * determined from the operands (unsupported operand kinds, out-of-range
 * indexes, invalid regular expressions): callers must then treat the result as
 * unknown, never as a default value.
 * </p>
 */
final class ConstantOperations {

	/**
	 * The kinds of constants whose Python equality is decided exactly.
	 * Constants of any other kind (lists, dictionaries, objects) are never
	 * compared, since their equality depends on contents or on user-defined
	 * {@code __eq__} methods.
	 */
	private enum Kind {
		STRING,
		NUMBER,
		NONE,
		OTHER
	}

	private ConstantOperations() {
	}

	/**
	 * Yields whether the given operator is evaluated by this class.
	 *
	 * @param operator the operator
	 *
	 * @return {@code true} if {@link #compute} or {@link #test} can evaluate
	 *             it
	 */
	static boolean handles(
			BinaryOperator operator) {
		return operator instanceof StringConcat
				|| operator instanceof StringSubstringToEnd
				|| isPredicate(operator);
	}

	/**
	 * Yields whether the given operator produces a boolean.
	 *
	 * @param operator the operator
	 *
	 * @return {@code true} for the string predicates, the equality
	 *             comparisons and the numeric orderings
	 */
	static boolean isPredicate(
			BinaryOperator operator) {
		return operator instanceof StringStartsWith
				|| operator instanceof StringEndsWith
				|| operator instanceof StringContains
				|| operator instanceof StringEquals
				|| operator instanceof StringMatches
				|| operator instanceof ComparisonEq
				|| operator instanceof ComparisonNe
				|| PythonNumbers.isOrdering(operator);
	}

	/**
	 * Computes the value of a non-boolean operation on two constants.
	 *
	 * @param operator the operator, one for which {@link #handles} holds and
	 *                     {@link #isPredicate} does not
	 * @param left     the first operand
	 * @param right    the second operand
	 *
	 * @return the resulting string, or empty if it cannot be determined
	 */
	static Optional<String> compute(
			BinaryOperator operator,
			Constant left,
			Constant right) {
		Object l = left.getValue();
		Object r = right.getValue();
		if (operator instanceof StringConcat && l instanceof String && r instanceof String)
			return Optional.of((String) l + (String) r);
		if (operator instanceof StringSubstringToEnd && l instanceof String && isIntegral(r)) {
			String string = (String) l;
			long begin = ((Number) r).longValue();
			if (begin >= 0 && begin <= string.length())
				return Optional.of(string.substring((int) begin));
		}
		return Optional.empty();
	}

	/**
	 * Evaluates a predicate on two constants.
	 *
	 * @param operator the operator, one for which {@link #isPredicate} holds
	 * @param left     the first operand
	 * @param right    the second operand
	 *
	 * @return the truth value of the predicate, or empty if it cannot be
	 *             determined
	 */
	static Optional<Boolean> test(
			BinaryOperator operator,
			Constant left,
			Constant right) {
		if (operator instanceof ComparisonEq)
			return pythonEquals(left, right);
		if (operator instanceof ComparisonNe)
			return pythonEquals(left, right).map(equal -> !equal);
		if (PythonNumbers.isOrdering(operator))
			return PythonNumbers.compare(operator, left, right);

		Object l = left.getValue();
		Object r = right.getValue();
		if (!(l instanceof String) || !(r instanceof String))
			return Optional.empty();
		String string = (String) l;
		String other = (String) r;
		if (operator instanceof StringStartsWith)
			return Optional.of(string.startsWith(other));
		if (operator instanceof StringEndsWith)
			return Optional.of(string.endsWith(other));
		if (operator instanceof StringContains)
			return Optional.of(string.contains(other));
		if (operator instanceof StringEquals)
			return Optional.of(string.equals(other));
		if (operator instanceof StringMatches)
			try {
				return Optional.of(string.matches(other));
			} catch (PatternSyntaxException e) {
				return Optional.empty();
			}
		return Optional.empty();
	}

	/**
	 * Decides {@code left == right} as Python does for strings, numbers,
	 * booleans and {@code None}: strings are equal when their contents are,
	 * numbers when their values are (booleans count as {@code 0} and
	 * {@code 1}, integers and floats compare by value), {@code None} only
	 * equals itself, and values of two different kinds among these are never
	 * equal.
	 *
	 * @param left  the first operand
	 * @param right the second operand
	 *
	 * @return the result of the comparison, or empty if an operand is of
	 *             another kind
	 */
	private static Optional<Boolean> pythonEquals(
			Constant left,
			Constant right) {
		Kind leftKind = kindOf(left);
		Kind rightKind = kindOf(right);
		if (leftKind == Kind.OTHER || rightKind == Kind.OTHER)
			return Optional.empty();
		if (leftKind != rightKind)
			return Optional.of(false);
		switch (leftKind) {
		case STRING:
			return Optional.of(left.getValue().equals(right.getValue()));
		case NUMBER:
			return PythonNumbers.equal(left, right);
		case NONE:
			return Optional.of(true);
		default:
			return Optional.empty();
		}
	}

	/**
	 * Decides whether a constant is true in a condition, as Python's
	 * {@code bool()} does: {@code False}, {@code None}, zero and the empty
	 * string are false; other booleans, numbers (not-a-number included) and
	 * strings are true.
	 *
	 * @param constant the constant
	 *
	 * @return its truth value, or empty for constants of other kinds
	 */
	static Optional<Boolean> truthiness(
			Constant constant) {
		if (constant instanceof PyNoneConstant || constant.getStaticType().isNullType())
			return Optional.of(false);
		Object value = constant.getValue();
		if (value instanceof Boolean)
			return Optional.of((Boolean) value);
		if (value instanceof String)
			return Optional.of(!((String) value).isEmpty());
		if (value instanceof Double || value instanceof Float)
			return Optional.of(((Number) value).doubleValue() != 0d);
		if (value instanceof Number)
			return Optional.of(((Number) value).longValue() != 0L);
		return Optional.empty();
	}

	private static Kind kindOf(
			Constant constant) {
		if (constant instanceof PyNoneConstant || constant.getStaticType().isNullType())
			return Kind.NONE;
		Object value = constant.getValue();
		if (value instanceof String)
			return Kind.STRING;
		if (value instanceof Number || value instanceof Boolean)
			return Kind.NUMBER;
		return Kind.OTHER;
	}

	private static boolean isIntegral(
			Object value) {
		return value instanceof Boolean
				|| value instanceof Byte
				|| value instanceof Short
				|| value instanceof Integer
				|| value instanceof Long;
	}
}
