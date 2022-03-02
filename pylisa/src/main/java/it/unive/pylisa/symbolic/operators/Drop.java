package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

public class Drop implements BinaryOperator, DataframeOperatorWithSideEffects {

    public static final Drop INSTANCE = new Drop();

    private Drop() {}

    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
        if (left.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(t -> t.equals(PyListType.INSTANCE)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
    }

    @Override
    public SymbolicExpression getDataFrame(SymbolicExpression container) {
        return ((BinaryExpression) container).getLeft();
    }

    @Override
    public String toString() {
        return "drop_columns->";
    }
    
}
