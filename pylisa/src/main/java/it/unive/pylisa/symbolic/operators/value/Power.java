package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.NumericOperation;

public class Power extends NumericOperation implements BinaryOperator {
	/**
	 * The singleton instance of this class.
	 */
	public static final Power INSTANCE = new Power();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected Power() {
	}

	@Override
	public String toString() {
		return "**";
	}
}
