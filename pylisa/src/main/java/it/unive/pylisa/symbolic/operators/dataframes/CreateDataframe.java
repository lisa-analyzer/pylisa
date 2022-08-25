package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class CreateDataframe implements UnaryOperator {

	public static final CreateDataframe INSTANCE = new CreateDataframe();

	private CreateDataframe() {
	}

	@Override
	public String toString() {
		return "create_df";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> argument) {
		if (argument.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.DICT))))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
	}
}
