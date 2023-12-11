package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class StringFormat implements BinaryOperator {
	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		if (left.stream().noneMatch(Type::isStringType) && right.stream().noneMatch(Type::isStringType))
			return Collections.emptySet();
		return Collections.singleton(StringType.INSTANCE);
	}
}