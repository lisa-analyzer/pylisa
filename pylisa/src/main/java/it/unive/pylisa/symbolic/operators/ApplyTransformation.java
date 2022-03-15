package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;

public class ApplyTransformation implements UnaryOperator, DataframeOperatorWithSideEffects {

	public enum Kind {
		UNKNOWN,
		BOTTOM,
		TO_DATETIME,
		TO_GEOCODE,
	}

	private final Kind type;

	public ApplyTransformation(Kind type) {
		this.type = type;
	}

	public Kind getKind() {
		return type;
	}

	@Override
	public String toString() {
		return "convert(" + type + ")";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> argument) {
		if (argument.noneMatch(t -> t.equals(PandasSeriesType.INSTANCE)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasSeriesType.INSTANCE);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((UnaryExpression) container).getExpression();
	}
}
