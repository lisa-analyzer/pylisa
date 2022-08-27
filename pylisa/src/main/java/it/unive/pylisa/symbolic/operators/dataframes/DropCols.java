package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class DropCols implements BinaryOperator {

	public static final DropCols INSTANCE = new DropCols();

	private DropCols() {
	}

	@Override
	public String toString() {
		return "drop_columns->";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.noneMatch(t -> t.equals(df)))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
	}
}
