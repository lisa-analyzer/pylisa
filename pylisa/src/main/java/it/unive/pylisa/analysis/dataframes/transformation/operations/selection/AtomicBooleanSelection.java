package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

public class AtomicBooleanSelection<T> extends BooleanSelection {

    public static enum Operator {
        EQ,
        NEQ,
        LEQ,
        GEQ,
        GT,
        LT
    }
    
    private String colName;
    private Operator op;
    private T val;

    public AtomicBooleanSelection(String colName, Operator op, T val) {
        this.colName = colName;
        this.op = op;
        this.val = val;
    }

    /**
     * @return String return the colName
     */
    public String getColName() {
        return colName;
    }

    /**
     * @return Operator return the op
     */
    public Operator getOp() {
        return op;
    }

    /**
     * @return T return the val
     */
    public T getVal() {
        return val;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        
        AtomicBooleanSelection o = (AtomicBooleanSelection) obj;

        return getColName().equals(o.colName) && getOp().equals(o.getOp()) && getVal().equals(o.getVal());
    }

}
