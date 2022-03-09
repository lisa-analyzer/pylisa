package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

public class ConcatRows implements BinaryOperator {

    public static final ConcatRows INSTANCE = new ConcatRows();

    private ConcatRows() {}

    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
        if (left.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
            return Caches.types().mkEmptySet();
        if (right.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
            return Caches.types().mkEmptySet();
        return Caches.types().mkSingletonSet(PandasDataframeType.INSTANCE);
    }

    @Override
    public String toString() {
        return "concat_rows->";
    }
    
}
