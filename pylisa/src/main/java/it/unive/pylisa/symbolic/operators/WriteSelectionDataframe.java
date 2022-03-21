package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;

public class WriteSelectionDataframe implements BinaryOperator, DataframeOperatorWithSideEffects {

	public static final WriteSelectionDataframe INSTANCE = new WriteSelectionDataframe();

	private WriteSelectionDataframe() {
	}

	@Override
	public String toString() {
		return "write";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PandasSeriesType.INSTANCE)))
			return Caches.types().mkEmptySet();
		if (left.noneMatch(t -> t.equals(PandasSeriesType.INSTANCE)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasSeriesType.INSTANCE);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((BinaryExpression) container).getLeft();
	}
}
