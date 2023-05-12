package it.unive.pylisa.symbolic.operators.compare;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.ComparisonOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

public class PyComparisonLe implements ComparisonOperator, BinaryOperator {

	public static final PyComparisonLe INSTANCE = new PyComparisonLe();

	private PyComparisonLe() {
	}

	@Override
	public String toString() {
		return "<=";
	}

	@Override
	public ComparisonOperator opposite() {
		return PyComparisonGt.INSTANCE;
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		return Collections.singleton(BoolType.INSTANCE);
	}
}
