package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ColumnAccess implements BinaryOperator, DataframeOperatorWithSideEffects {

	public static final ColumnAccess INSTANCE = new ColumnAccess();

	private ColumnAccess() {
	}

	@Override
	public String toString() {
		return "->";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(Type::isStringType)
				|| right.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES));
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((BinaryExpression) container).getLeft();
	}
}
