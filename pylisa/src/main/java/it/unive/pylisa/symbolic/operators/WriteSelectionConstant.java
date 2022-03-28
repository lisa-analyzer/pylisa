package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;

public class WriteSelectionConstant implements BinaryOperator, DataframeOperatorWithSideEffects {

    public static final WriteSelectionConstant INSTANCE = new WriteSelectionConstant();

    private WriteSelectionConstant() {
    }
    
    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
        if (left.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE) || t.equals(PandasSeriesType.INSTANCE)))
            return Caches.types().mkEmptySet();
        if (right.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE) || t.equals(PandasSeriesType.INSTANCE) || t.isNumericType() || t.isStringType()))
            return Caches.types().mkEmptySet();
        return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
    }

    @Override
    public String toString() {
        return "write_selection->";
    }

    @Override
    public SymbolicExpression getDataFrame(SymbolicExpression container) {
        return ((BinaryExpression) container).getLeft();
    }
}