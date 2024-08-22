package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;

public class Assign<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final DataframeSelection<R, C> selection;

	public Assign(
			CodeLocation where,
			int index,
			DataframeSelection<R, C> selection) {
		super(where, index);
		this.selection = selection;
	}

	public DataframeSelection<R, C> getSelection() {
		return selection;
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Assign<?, ?> o = (Assign<?, ?>) other;
		return selection.lessOrEqual(o.selection);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Assign<?, ?> o = (Assign<?, ?>) other;
		return new Assign<>(where, index, selection.lub(o.selection));
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		Assign<?, ?> other = (Assign<?, ?>) o;
		return selection.compareTo(other.selection);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Assign<?, ?> other = (Assign<?, ?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "assign_to:" + selection;
	}
}
