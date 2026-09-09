package it.unive.pylisa.symbolic.operators.rclpy.node;

import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

public class NewNode implements BinaryOperator {

	public String toString() {
		return "new_rclpy_node";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return left;
	}
}
