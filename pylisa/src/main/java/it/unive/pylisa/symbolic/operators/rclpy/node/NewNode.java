package it.unive.pylisa.symbolic.operators.rclpy.node;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

import java.beans.Expression;
import java.util.Set;

public class NewNode implements BinaryOperator {

    public String toString() {
        return "new_rclpy_node";
    }

    @Override
    public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
        return left;
    }
}
