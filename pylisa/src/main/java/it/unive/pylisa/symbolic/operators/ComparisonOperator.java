package it.unive.pylisa.symbolic.operators;

import java.util.HashMap;
import java.util.Map;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;

public class ComparisonOperator extends BaseLattice<ComparisonOperator> {

    public static enum Operator {
        EQ,
        NEQ,
        LT,
        LEQ,
        GT,
        GEQ,
        TOP,
        BOT
    }

    private static final Map<Operator, String> opStrings = new HashMap<>();
    static {
        opStrings.put(Operator.EQ, "==");
        opStrings.put(Operator.NEQ, "!=");
        opStrings.put(Operator.GT, ">");
        opStrings.put(Operator.GEQ, ">=");
        opStrings.put(Operator.LT, "<");
        opStrings.put(Operator.LEQ, "<=");
        opStrings.put(Operator.TOP, "T");
        opStrings.put(Operator.BOT, "_|_");
    }

    private Operator op;

    public ComparisonOperator(Operator op) {
        this.op = op;
    }

    public static final ComparisonOperator EQ = new ComparisonOperator(Operator.EQ);
    public static final ComparisonOperator NEQ = new ComparisonOperator(Operator.NEQ);
    public static final ComparisonOperator GT = new ComparisonOperator(Operator.GT);
    public static final ComparisonOperator GEQ = new ComparisonOperator(Operator.GEQ);
    public static final ComparisonOperator LT = new ComparisonOperator(Operator.LT);
    public static final ComparisonOperator LEQ = new ComparisonOperator(Operator.LEQ);
    public static final ComparisonOperator TOP = new ComparisonOperator(Operator.TOP);
    public static final ComparisonOperator BOT = new ComparisonOperator(Operator.BOT);

    @Override
    public ComparisonOperator top() {
        return new ComparisonOperator(Operator.TOP);
    }

    @Override
    public ComparisonOperator bottom() {
        return new ComparisonOperator(Operator.TOP);
    }

    @Override
    protected ComparisonOperator lubAux(ComparisonOperator other) throws SemanticException {
        if (lessOrEqual(other)) {
            return other;
        }
        return top();
    }

    @Override
    protected ComparisonOperator wideningAux(ComparisonOperator other) throws SemanticException {
        return lub(other);
    }

    @Override
    protected boolean lessOrEqualAux(ComparisonOperator other) throws SemanticException {
        if (this.op == Operator.BOT) {
            return true;
        } else if (this.op == other.op) {
            return true;
        }
        return false;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null || !(obj instanceof ComparisonOperator))
            return false;
        ComparisonOperator o = (ComparisonOperator) obj;
        if (op != o.op)
            return false;
        return true;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
		int result = 1;
		result = prime * result + ((op == null) ? 0 : op.hashCode());
		return result;
    }

    @Override
    public String toString() {
        return opStrings.getOrDefault(op, "T");
    }
    
}
