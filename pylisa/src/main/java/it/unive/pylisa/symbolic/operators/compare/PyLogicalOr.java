package it.unive.pylisa.symbolic.operators.compare;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.LogicalOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

public class PyLogicalOr implements LogicalOperator, BinaryOperator {

	public static final PyLogicalOr INSTANCE = new PyLogicalOr();

	private PyLogicalOr() {
	}

	@Override
	public String toString() {
		return "||";
	}

	@Override
	public LogicalOperator opposite() {
		return PyLogicalAnd.INSTANCE;
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		return Collections.singleton(BoolType.INSTANCE);
	}
}