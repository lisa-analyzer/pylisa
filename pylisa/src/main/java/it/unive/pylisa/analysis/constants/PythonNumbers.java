package it.unive.pylisa.analysis.constants;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.ModuloOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.RemainderOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.Optional;

/**
 * The concrete semantics of Python's arithmetic and ordering on numeric
 * constants, as {@link ConstantPropagation} evaluates them.
 * <p>
 * Python integers are unbounded; here they are represented as {@link Integer}
 * when they fit 32 bits and as {@link Long} otherwise, and a result that does
 * not fit 64 bits has no representation: it is reported as unknown, never as a
 * wrapped-around value. Booleans count as the integers {@code 0} and
 * {@code 1}. Python floats are IEEE doubles and are represented as
 * {@link Double}; single-precision values are widened. Operations on floats
 * follow Java's double arithmetic, which is IEEE's, as CPython's is.
 * </p>
 * <p>
 * Every method yields an empty result when the concrete outcome cannot be
 * determined from the operands: callers must then treat the result as unknown.
 * </p>
 */
final class PythonNumbers {

	private static final BigInteger MIN_LONG = BigInteger.valueOf(Long.MIN_VALUE);

	private static final BigInteger MAX_LONG = BigInteger.valueOf(Long.MAX_VALUE);

	private PythonNumbers() {
	}

	/**
	 * Yields whether the given operator is an ordering comparison evaluated by
	 * {@link #compare}.
	 *
	 * @param operator the operator
	 *
	 * @return {@code true} for {@code <}, {@code <=}, {@code >} and {@code >=}
	 */
	static boolean isOrdering(
			BinaryOperator operator) {
		return operator instanceof PyComparisonLt || operator instanceof ComparisonLt
				|| operator instanceof PyComparisonLe || operator instanceof ComparisonLe
				|| operator instanceof PyComparisonGt || operator instanceof ComparisonGt
				|| operator instanceof PyComparisonGe || operator instanceof ComparisonGe;
	}

	/**
	 * Yields whether a constant is a number (a boolean included).
	 *
	 * @param constant the constant
	 *
	 * @return {@code true} if it is
	 */
	static boolean isNumber(
			Constant constant) {
		return isIntegral(constant.getValue()) || isFloat(constant.getValue());
	}

	/**
	 * Yields whether a constant is a numeric zero, which makes a division or a
	 * remainder raise {@code ZeroDivisionError}.
	 *
	 * @param constant the constant
	 *
	 * @return {@code true} if it is
	 */
	static boolean isZero(
			Constant constant) {
		Object value = constant.getValue();
		if (isIntegral(value))
			return asLong(value) == 0;
		return isFloat(value) && ((Number) value).doubleValue() == 0d;
	}

	/**
	 * Computes an arithmetic operation between two numbers: {@code +},
	 * {@code -}, {@code *}, {@code /}, {@code %} or {@code **}.
	 *
	 * @param operator the operator
	 * @param left     the first operand
	 * @param right    the second operand
	 * @param location the location of the result
	 *
	 * @return the result, or empty if it cannot be determined (including a
	 *             zero divisor)
	 */
	static Optional<Constant> arithmetic(
			BinaryOperator operator,
			Constant left,
			Constant right,
			CodeLocation location) {
		Object l = left.getValue();
		Object r = right.getValue();
		if (!isNumber(left) || !isNumber(right))
			return Optional.empty();
		if (operator instanceof DivisionOperator) {
			if (isZero(right))
				return Optional.empty();
			// Python divides integers exactly and rounds once: dividing their
			// double approximations rounds the same way only when both are
			// exactly representable
			if ((isIntegral(l) && !exactlyDouble(asLong(l))) || (isIntegral(r) && !exactlyDouble(asLong(r))))
				return Optional.empty();
			return real(asDouble(l) / asDouble(r), location);
		}
		if (isIntegral(l) && isIntegral(r))
			return integralArithmetic(operator, asLong(l), asLong(r), location);
		double a = asDouble(l);
		double b = asDouble(r);
		if (operator instanceof AdditionOperator)
			return real(a + b, location);
		if (operator instanceof SubtractionOperator)
			return real(a - b, location);
		if (operator instanceof MultiplicationOperator)
			return real(a * b, location);
		if (operator instanceof RemainderOperator || operator instanceof ModuloOperator)
			return b == 0d ? Optional.empty() : real(floatModulo(a, b), location);
		if (operator instanceof Power)
			return floatPower(a, b, location);
		return Optional.empty();
	}

	/**
	 * Computes {@code a ** b} on floats only when the result is exact: a
	 * finite integral base and a small non-negative integral exponent whose
	 * power is representable as a double. CPython delegates every other case
	 * to the platform's {@code pow}, whose rounding and special cases (complex
	 * results, overflow errors) the analysis does not reproduce.
	 */
	private static Optional<Constant> floatPower(
			double a,
			double b,
			CodeLocation location) {
		if (!Double.isFinite(a) || !Double.isFinite(b) || a != Math.rint(a) || b != Math.rint(b) || b < 0
				|| b > 64)
			return Optional.empty();
		if (b == 0)
			return real(1d, location);
		if (a == 0)
			// the sign of a zero result depends on the sign of the base
			return Optional.empty();
		BigInteger power = new BigDecimal(a).toBigIntegerExact().pow((int) b);
		BigInteger magnitude = power.abs();
		int significantBits = magnitude.bitLength() - magnitude.getLowestSetBit();
		if (significantBits > 53 || magnitude.bitLength() > 1024)
			return Optional.empty();
		return real(power.doubleValue(), location);
	}

	private static boolean exactlyDouble(
			long value) {
		return Math.abs(value) <= (1L << 53);
	}

	private static Optional<Constant> integralArithmetic(
			BinaryOperator operator,
			long a,
			long b,
			CodeLocation location) {
		BigInteger x = BigInteger.valueOf(a);
		BigInteger y = BigInteger.valueOf(b);
		if (operator instanceof AdditionOperator)
			return integer(x.add(y), location);
		if (operator instanceof SubtractionOperator)
			return integer(x.subtract(y), location);
		if (operator instanceof MultiplicationOperator)
			return integer(x.multiply(y), location);
		if (operator instanceof RemainderOperator || operator instanceof ModuloOperator)
			// Python's remainder has the sign of the divisor
			return b == 0 ? Optional.empty() : integer(BigInteger.valueOf(Math.floorMod(a, b)), location);
		if (operator instanceof Power) {
			if (b < 0)
				// a negative exponent gives a float computed by the platform's
				// pow, and 0 ** -n raises
				return Optional.empty();
			if (b == 0)
				return integer(BigInteger.ONE, location);
			if (x.abs().compareTo(BigInteger.ONE) <= 0)
				// 0, 1 and -1 keep their magnitude
				return integer(b % 2 == 0 ? x.abs() : x, location);
			// |x| >= 2, so any exponent above 64 exceeds 64 bits
			if (b > 64)
				return Optional.empty();
			return integer(x.pow((int) b), location);
		}
		return Optional.empty();
	}

	/**
	 * Computes Python's {@code x % y} on floats: the result has the sign of
	 * the divisor.
	 */
	private static double floatModulo(
			double x,
			double y) {
		double mod = x % y;
		if (mod == 0d)
			// a zero remainder has the sign of the divisor
			return Math.copySign(0d, y);
		if ((mod < 0d) != (y < 0d))
			mod += y;
		return mod;
	}

	/**
	 * Computes the arithmetic negation of a number.
	 *
	 * @param argument the number
	 * @param location the location of the result
	 *
	 * @return the result, or empty if it cannot be determined
	 */
	static Optional<Constant> negate(
			Constant argument,
			CodeLocation location) {
		Object value = argument.getValue();
		if (isIntegral(value))
			return integer(BigInteger.valueOf(asLong(value)).negate(), location);
		if (isFloat(value))
			return real(-asDouble(value), location);
		return Optional.empty();
	}

	/**
	 * Computes {@code math.floor} of a number, an integer.
	 *
	 * @param argument the number
	 * @param location the location of the result
	 *
	 * @return the result, or empty if it cannot be determined (infinite or
	 *             not-a-number floats, which make Python raise)
	 */
	static Optional<Constant> floor(
			Constant argument,
			CodeLocation location) {
		Object value = argument.getValue();
		if (isIntegral(value))
			return integer(BigInteger.valueOf(asLong(value)), location);
		if (!isFloat(value))
			return Optional.empty();
		double real = asDouble(value);
		if (Double.isNaN(real) || Double.isInfinite(real))
			return Optional.empty();
		return integer(new BigDecimal(Math.floor(real)).toBigInteger(), location);
	}

	/**
	 * Decides an ordering comparison between two numbers.
	 *
	 * @param operator the operator, one for which {@link #isOrdering} holds
	 * @param left     the first operand
	 * @param right    the second operand
	 *
	 * @return the truth value, or empty if an operand is not a number or is
	 *             a float that is not a number
	 */
	static Optional<Boolean> compare(
			BinaryOperator operator,
			Constant left,
			Constant right) {
		if (!isNumber(left) || !isNumber(right))
			return Optional.empty();
		Object l = left.getValue();
		Object r = right.getValue();
		int order;
		if (isIntegral(l) && isIntegral(r))
			order = Long.compare(asLong(l), asLong(r));
		else {
			double a = asDouble(l);
			double b = asDouble(r);
			if (Double.isNaN(a) || Double.isNaN(b))
				// every ordering with NaN is false, so that the negation of an
				// ordering is not its opposite: deciding nothing keeps the
				// refinement of both branches of a condition sound
				return Optional.empty();
			// integers and floats are compared exactly, as Python does
			order = exactOrder(l, r);
		}
		if (operator instanceof PyComparisonLt || operator instanceof ComparisonLt)
			return Optional.of(order < 0);
		if (operator instanceof PyComparisonLe || operator instanceof ComparisonLe)
			return Optional.of(order <= 0);
		if (operator instanceof PyComparisonGt || operator instanceof ComparisonGt)
			return Optional.of(order > 0);
		return Optional.of(order >= 0);
	}

	/**
	 * Decides {@code left == right} on two numbers as Python does: integers
	 * and floats compare by their exact values.
	 *
	 * @param left  the first number
	 * @param right the second number
	 *
	 * @return the result of the comparison, or empty if an operand is not a
	 *             number
	 */
	static Optional<Boolean> equal(
			Constant left,
			Constant right) {
		if (!isNumber(left) || !isNumber(right))
			return Optional.empty();
		Object l = left.getValue();
		Object r = right.getValue();
		if (isIntegral(l) && isIntegral(r))
			return Optional.of(asLong(l) == asLong(r));
		if (Double.isNaN(asDouble(l)) || Double.isNaN(asDouble(r)))
			return Optional.of(false);
		return Optional.of(exactOrder(l, r) == 0);
	}

	private static int exactOrder(
			Object left,
			Object right) {
		double a = asDouble(left);
		double b = asDouble(right);
		if (Double.isInfinite(a) || Double.isInfinite(b))
			return Double.compare(a, b);
		return exact(left).compareTo(exact(right));
	}

	private static BigDecimal exact(
			Object value) {
		return isIntegral(value) ? new BigDecimal(asLong(value)) : new BigDecimal(asDouble(value));
	}

	private static Optional<Constant> integer(
			BigInteger value,
			CodeLocation location) {
		if (value.compareTo(MIN_LONG) < 0 || value.compareTo(MAX_LONG) > 0)
			return Optional.empty();
		long result = value.longValue();
		if (result >= Integer.MIN_VALUE && result <= Integer.MAX_VALUE)
			return Optional.of(new Constant(Int32Type.INSTANCE, (int) result, location));
		return Optional.of(new Constant(Int64Type.INSTANCE, result, location));
	}

	private static Optional<Constant> real(
			double value,
			CodeLocation location) {
		return Optional.of(new Constant(Float64Type.INSTANCE, value, location));
	}

	private static boolean isIntegral(
			Object value) {
		return value instanceof Boolean
				|| value instanceof Byte
				|| value instanceof Short
				|| value instanceof Integer
				|| value instanceof Long;
	}

	private static boolean isFloat(
			Object value) {
		return value instanceof Float || value instanceof Double;
	}

	private static long asLong(
			Object value) {
		if (value instanceof Boolean)
			return ((Boolean) value) ? 1L : 0L;
		return ((Number) value).longValue();
	}

	private static double asDouble(
			Object value) {
		if (value instanceof Boolean)
			return ((Boolean) value) ? 1d : 0d;
		return ((Number) value).doubleValue();
	}
}
