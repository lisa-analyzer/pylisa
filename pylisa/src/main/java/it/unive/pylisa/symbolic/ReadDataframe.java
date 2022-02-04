package it.unive.pylisa.symbolic;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.PyDataframeType;

public class ReadDataframe implements UnaryOperator {

	public static final ReadDataframe INSTANCE = new ReadDataframe();

	private ReadDataframe() {
	}

	@Override
	public String toString() {
		return "read_df";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> argument) {
		if (argument.noneMatch(Type::isStringType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyDataframeType.INSTANCE);
	}
}
