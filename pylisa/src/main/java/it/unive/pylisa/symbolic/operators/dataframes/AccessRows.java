package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class AccessRows implements TernaryOperator {

	public static final AccessRows INSTANCE = new AccessRows();

	private AccessRows() {
	}

	@Override
	public String toString() {
		return "rows->";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.noneMatch(t -> t.equals(df)))
			return Caches.types().mkEmptySet();
		if (middle.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
	}
}
