package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.Selection;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;

public class BooleanComparison<S extends Selection<S>> extends DataframeOperation {

    private S selection;
    private ComparisonOperator op;
    private ConstantPropagation value;

    public BooleanComparison(CodeLocation where, S selection, ComparisonOperator op, ConstantPropagation value) {
        super(where);
        this.selection = selection;
        this.op = op;
        this.value = value;
    }

    @Override
    protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
        BooleanComparison<?> o = (BooleanComparison<?>) other;
        if (this.equals(o))
            return true;
        if (!this.selection.getClass().equals(o.selection.getClass()))
            return false;
        return selection.lessOrEqual((S) o.selection) && value.lessOrEqual(o.value);
    }

    @Override
    protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
        BooleanComparison<?> o = (BooleanComparison<?>) other;
        if (this.op != o.op || !this.selection.getClass().equals(o.selection.getClass())) {
            return DataframeOperation.TOP;
        }
        return new BooleanComparison<>(SyntheticLocation.INSTANCE, selection.lub((S) o.selection), op, value.lub(o.value));
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        BooleanComparison<?> other = (BooleanComparison<?>) obj;
        if (selection == null) {
            if (other.selection != null)
                return false;
        } else if (!selection.equals(other.selection))
            return false;

        if (op == null) {
            if (other.op != null)
                return false;
        } else if (!op.equals(other.op))
            return false;
        
        if (value == null) {
            if (other.value != null)
                return false;
        } else if (!value.equals(other.value))
            return false;

        return true;
    }

    @Override
    public String toString() {
        return op.toString() + " " + value.toString();
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((value == null) ? 0 : value.hashCode());
        return result;
    }
}
