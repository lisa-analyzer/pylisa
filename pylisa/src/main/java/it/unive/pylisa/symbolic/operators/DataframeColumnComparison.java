package it.unive.pylisa.symbolic.operators;

import java.util.HashMap;
import java.util.Map;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;

public class DataframeColumnComparison implements TernaryOperator{

    public static enum Operator {
        EQ,
        NEQ,
        GT,
        GEQ,
        LT,
        LEQ
    }

    private static final Map<Operator, String> opStrings = new HashMap<>();
    static {
        opStrings.put(Operator.EQ, "== ->");
        opStrings.put(Operator.NEQ, "!= ->");
        opStrings.put(Operator.GT, "> ->");
        opStrings.put(Operator.GEQ, ">= ->");
        opStrings.put(Operator.LT, "< ->");
        opStrings.put(Operator.LEQ, "<= ->");
    }

    private final Operator op;

    public DataframeColumnComparison(Operator op) {
        this.op = op;
    }

    @Override
    public String toString() {
        return opStrings.getOrDefault(op, "series_comp ->");
    }

    public Operator getOp() {
        return this.op;
    }

    @Override
    public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
        if (left.noneMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
            return Caches.types().mkEmptySet();
        if (middle.noneMatch(t -> t.isStringType()))
            // col name
            return Caches.types().mkEmptySet();
        if (right.noneMatch(t -> t.isNumericType() || t.isStringType()))
            return Caches.types().mkEmptySet();
        return Caches.types().mkSingletonSet(PandasSeriesType.INSTANCE);
    }
    
}
