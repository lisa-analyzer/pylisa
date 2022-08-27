package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class AccessRowsColumns implements TernaryOperator {

	public static final AccessRowsColumns INSTANCE = new AccessRowsColumns();

	private AccessRowsColumns() {
	}

	@Override
	public String toString() {
		return "access_rows_cols->";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Caches.types().mkEmptySet();
		if (middle.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.SLICE))
				|| t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES))))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.SLICE))
				|| t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
	}
}
