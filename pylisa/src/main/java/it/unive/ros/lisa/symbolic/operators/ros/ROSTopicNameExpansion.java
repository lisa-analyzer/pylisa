package it.unive.ros.lisa.symbolic.operators.ros;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

import java.util.Collections;
import java.util.Set;

public class ROSTopicNameExpansion implements TernaryOperator {
    @Override
    public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> middle, Set<Type> right) {
        if (left.stream().noneMatch(Type::isStringType) && middle.stream().noneMatch(Type::isStringType) && right.stream().noneMatch(Type::isStringType))
            return Collections.emptySet();
        return Collections.singleton(StringType.INSTANCE);
    }
}
