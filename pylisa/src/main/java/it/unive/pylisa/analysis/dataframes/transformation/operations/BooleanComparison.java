package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.BooleanSelection;

@SuppressWarnings("unchecked")
public class BooleanComparison<B extends BooleanSelection<B>> extends DataframeOperation {

    private B selection;

    public BooleanComparison(CodeLocation where, B selection) {
        super(where);
        this.selection = selection;
    }

	@Override
    protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
        BooleanComparison<?> o = (BooleanComparison<?>) other;
        if (this.equals(o))
            return true;
        if (!this.selection.getClass().equals(o.selection.getClass()))
            return false;
        return selection.lessOrEqual((B) o.selection);
    }

    @Override
    protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
        BooleanComparison<?> o = (BooleanComparison<?>) other;
        if (!this.selection.getClass().equals(o.selection.getClass())) {
            return DataframeOperation.TOP;
        }
        return new BooleanComparison<>(SyntheticLocation.INSTANCE, selection.lub((B) o.selection));
    }

    @Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		BooleanComparison<?> other = (BooleanComparison<?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		return true;
	}

	@Override
    public String toString() {
        return selection.toString();
    }

    public B getSelection() {
        return selection;
    }
}
