package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.VoidType;
import it.unive.lisa.util.collections.externalSet.ExternalSet;

public class SetOption implements BinaryOperator {

	public static final SetOption INSTANCE = new SetOption();

	private SetOption() {
	}

	@Override
	public String toString() {
		return "set_option";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		if (left.noneMatch(Type::isStringType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(VoidType.INSTANCE);
	}
}
