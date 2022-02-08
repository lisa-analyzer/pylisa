package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.PandasDataframeType;

public class SetOptionAux implements TernaryOperator, DataframeOperatorWithSideEffects {

	public static final SetOptionAux INSTANCE = new SetOptionAux();

	private SetOptionAux() {
	}

	@Override
	public String toString() {
		return "set_option";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PandasDataframeType.REFERENCE)))
			return Caches.types().mkEmptySet();
		if (middle.noneMatch(Type::isStringType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((TernaryExpression) container).getLeft();
	}
}
