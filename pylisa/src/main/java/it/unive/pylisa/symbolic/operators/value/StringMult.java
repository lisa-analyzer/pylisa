package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.symbolic.value.operator.binary.StringOperation;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.PythonArithmetic;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class StringMult extends StringOperation {
	public static final StringMult INSTANCE = new StringMult();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected StringMult() {
	}

	@Override
	public String toString() {
		return "*";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		// the count may be a boolean, which Python treats as the integer 0 or 1
		Set<Type> leftCounts = PythonArithmetic.asNumbers(left);
		Set<Type> rightCounts = PythonArithmetic.asNumbers(right);
		if ((leftCounts.stream().anyMatch(Type::isNumericType) && right.stream().anyMatch(Type::isStringType)) ||
				(left.stream().anyMatch(Type::isStringType) && rightCounts.stream().anyMatch(Type::isNumericType))) {
			return Collections.singleton(resultType(types));
		}

		return Collections.emptySet();

	}

	@Override
	protected Type resultType(
			TypeSystem types) {
		return types.getStringType();
	}
}
