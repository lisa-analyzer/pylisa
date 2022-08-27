package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class WriteSelectionConstant implements BinaryOperator {

	public static final WriteSelectionConstant INSTANCE = new WriteSelectionConstant();

	private WriteSelectionConstant() {
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.noneMatch(t -> t.equals(df) || t.equals(series)))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(t -> t.equals(df) || t.equals(series) || t.isNumericType() || t.isStringType()))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
	}

	@Override
	public String toString() {
		return "write_selection->";
	}
}
