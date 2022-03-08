package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

public class BooleanSelectionExpression extends BooleanSelection {
    public static enum Type {
        AND,
        OR
    }

    private Type type;
    private BooleanSelection bs1, bs2;

    public BooleanSelectionExpression(Type type, BooleanSelection bs1, BooleanSelection bs2) {
        this.type = type;
        this.bs1 = bs1;
        this.bs2 = bs2;
    }

    /**
     * @return Type return the type
     */
    public Type getType() {
        return type;
    }

    public BooleanSelection getLeft() {
        return bs1;
    }

    public BooleanSelection getRight() {
        return bs2;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof BooleanSelectionExpression))
            return false;

        BooleanSelectionExpression exp = (BooleanSelectionExpression) obj;

        return getType().equals(exp.getType()) && 
            ((getLeft().equals(exp.getLeft()) && getRight().equals(exp.getRight())) ||
             (getLeft().equals(exp.getRight()) && getRight().equals(exp.getLeft())));
    }
}
