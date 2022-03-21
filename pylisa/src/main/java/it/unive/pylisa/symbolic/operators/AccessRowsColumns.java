package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.cfg.type.PySliceType;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;

public class AccessRowsColumns implements TernaryOperator, DataframeOperatorWithSideEffects {

    public static final AccessRowsColumns INSTANCE = new AccessRowsColumns();

    private AccessRowsColumns() {
    }

    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
        if (middle.noneMatch(t -> t.equals(PySliceType.INSTANCE) || t.equals(PandasSeriesType.INSTANCE)))
            return Caches.types().mkEmptySet();
        if (right.noneMatch(t -> t.equals(PyListType.INSTANCE)))
            return Caches.types().mkEmptySet();
        return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
    }

    @Override
    public String toString() {
        return "access_rows_cols->";
    }

    @Override
    public SymbolicExpression getDataFrame(SymbolicExpression container) {
        return ((TernaryExpression) container).getLeft();
    }

}
