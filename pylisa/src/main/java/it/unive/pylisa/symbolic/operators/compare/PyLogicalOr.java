package it.unive.pylisa.symbolic.operators.compare;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.LogicalOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.util.collections.externalSet.ExternalSet;

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
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		return Caches.types().mkSingletonSet(BoolType.INSTANCE);
	}
}