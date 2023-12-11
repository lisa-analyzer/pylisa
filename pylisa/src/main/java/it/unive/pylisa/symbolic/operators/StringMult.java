package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.StringOperator;
import it.unive.lisa.symbolic.value.operator.binary.StringOperation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class StringMult extends StringOperation implements StringOperator {
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
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		if ((left.stream().anyMatch(Type::isNumericType) && right.stream().anyMatch(Type::isStringType)) ||
				(left.stream().anyMatch(Type::isStringType) && right.stream().anyMatch(Type::isNumericType))) {
			return Collections.singleton(resultType(types));
		}

		return Collections.emptySet();

	}

	@Override
	protected Type resultType(TypeSystem types) {
		return types.getStringType();
	}
}