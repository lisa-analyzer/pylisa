package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

import java.util.Set;

public class LazyEvaluatedExpressionOperator implements UnaryOperator {
    public static LazyEvaluatedExpressionOperator INSTANCE = new LazyEvaluatedExpressionOperator();
    @Override
    public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
        return Set.of();
    }
}
