package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.binary.NumericOperation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Set;

public class Power extends NumericOperation {
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

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return super.typeInference(types, PythonArithmetic.asNumbers(left), PythonArithmetic.asNumbers(right));
	}
}