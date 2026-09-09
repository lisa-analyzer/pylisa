package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.binary.NumericOperation;

/**
 * Python's floor division ({@code a // b}), i.e. {@code floor(a / b)} rather
 * than truncation towards zero (which is what Java's {@code /} does on
 * integers). Kept distinct from true division since the result differs for
 * negative operands (e.g. {@code -7 // 2 == -4} in Python, not {@code -3}).
 */
public class FloorDivision extends NumericOperation {

	/**
	 * The singleton instance of this class.
	 */
	public static final FloorDivision INSTANCE = new FloorDivision();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected FloorDivision() {
	}

	@Override
	public String toString() {
		return "//";
	}
}
