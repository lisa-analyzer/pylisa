package it.unive.pylisa.symbolic.operators.compare;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.LogicalOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class PyLogicalAnd implements LogicalOperator, BinaryOperator {

	public static final PyLogicalAnd INSTANCE = new PyLogicalAnd();

	private PyLogicalAnd() {
	}

	@Override
	public String toString() {
		return "&&";
	}

	@Override
	public LogicalOperator opposite() {
		return PyLogicalOr.INSTANCE;
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		return Collections.singleton(BoolType.INSTANCE);
	}
}
