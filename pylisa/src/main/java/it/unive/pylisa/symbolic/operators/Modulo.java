package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.binary.NumericOperation;

/**
 * Python's {@code %} on numbers, i.e. {@code a - floor(a / b) * b}: the result
 * takes the sign of the divisor. Kept distinct from Java's {@code %} (emitted
 * by the SDK's {@code Remainder}/{@code NumericNonOverflowingRem}), which takes
 * the sign of the dividend instead (e.g. {@code -7 % 3 == 2} in Python, not
 * {@code -1}).
 */
public class Modulo extends NumericOperation {

	/**
	 * The singleton instance of this class.
	 */
	public static final Modulo INSTANCE = new Modulo();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected Modulo() {
	}

	@Override
	public String toString() {
		return "%";
	}
}
