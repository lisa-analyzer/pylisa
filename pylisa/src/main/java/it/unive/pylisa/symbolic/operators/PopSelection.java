package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

public class PopSelection implements UnaryOperator, DataframeOperatorWithSideEffects {

	public static final PopSelection INSTANCE = new PopSelection();

	private PopSelection() {
	}

	@Override
	public String toString() {
		return "pop_selection";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> argument) {
		if (argument.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((UnaryExpression) container).getExpression();
	}
}
