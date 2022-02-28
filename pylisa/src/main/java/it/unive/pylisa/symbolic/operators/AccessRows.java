package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

public class AccessRows implements TernaryOperator, DataframeOperatorWithSideEffects {

	public static final AccessRows INSTANCE = new AccessRows();

	private AccessRows() {
	}

	@Override
	public String toString() {
		return "rows->";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
			return Caches.types().mkEmptySet();
		if (middle.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((TernaryExpression) container).getLeft();
	}
}
