package it.unive.ros.lisa.symbolic.operators.ros;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class ROSFixNamespace implements UnaryOperator {
	@Override
	public Set<Type> typeInference(TypeSystem typeSystem, Set<Type> set) {
		if (set.stream().noneMatch(Type::isStringType))
			return Collections.emptySet();
		return Collections.singleton(StringType.INSTANCE);
	}
}
