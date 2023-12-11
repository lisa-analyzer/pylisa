package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.symbolic.value.operator.StringOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class StringConstructor implements StringOperator, UnaryOperator {
	public static final StringConstructor INSTANCE = new StringConstructor();

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
		if (argument.stream().noneMatch(Type::isStringType) && argument.stream().noneMatch(Type::isNumericType)) {
			return Collections.emptySet();
		}
		return Collections.singleton(types.getStringType());
	}

	@Override
	public String toString() {
		return "str";
	}
}
