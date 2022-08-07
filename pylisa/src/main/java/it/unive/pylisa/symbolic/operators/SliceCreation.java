package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class SliceCreation implements TernaryOperator {

    public static final SliceCreation INSTANCE = new SliceCreation();

    private SliceCreation() {
    }

    @Override
    public String toString() {
        return "new_slice";
    }

    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
    	ReferenceType seriesref = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES).getReference();
        if (left.noneMatch(t -> t.isNumericType() || t.isNullType() || t.equals(seriesref)))
            return Caches.types().mkEmptySet();
        if (middle.noneMatch(t -> t.isNumericType() || t.isNullType() || t.equals(seriesref)))
            return Caches.types().mkEmptySet();
        if (right.noneMatch(t -> t.isNumericType() || t.isNullType()))
            return Caches.types().mkEmptySet();
        return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.SLICE));
    }
    
}
