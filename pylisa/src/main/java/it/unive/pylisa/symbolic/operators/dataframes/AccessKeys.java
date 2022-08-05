package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class AccessKeys implements UnaryOperator {

	public static final AccessKeys INSTANCE = new AccessKeys();

	private AccessKeys() {
	}

	@Override
	public String toString() {
		return "keys()";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> arg) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (arg.noneMatch(t -> t.equals(df)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
	}
}
