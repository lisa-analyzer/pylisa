package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;

public class Project<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final DataframeSelection<R, C> selection;

	public Project(CodeLocation where, int index, DataframeSelection<R, C> selection) {
		super(where, index);
		this.selection = selection;
	}

	public Project(CodeLocation where, int index, C selection) {
		super(where, index);
		this.selection = new DataframeSelection<>(selection);
	}

	public Project(CodeLocation where, int index, R selection) {
		super(where, index);
		this.selection = new DataframeSelection<>(selection);
	}

	public Project(CodeLocation where, int index, R rows, C columns) {
		super(where, index);
		this.selection = new DataframeSelection<>(rows, columns);
	}

	@Override
	public String toString() {
		return "project:" + selection;
	}

	public DataframeSelection<R, C> getSelection() {
		return selection;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		Project<?, ?> o = (Project<?, ?>) other;
		return selection.lessOrEqual(o.selection);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		Project<?, ?> o = (Project<?, ?>) other;
		return new Project<>(where, index, selection.lub(o.selection));
	}

	@Override
	protected DataframeOperation wideningSameOperation(DataframeOperation other) throws SemanticException {
		Project<?, ?> o = (Project<?, ?>) other;
		return new Project<>(where, index, selection.lub(o.selection));
	}

	@Override
	protected int compareToSameOperation(DataframeOperation o) {
		Project<?, ?> other = (Project<?, ?>) o;
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
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Project<?, ?> other = (Project<?, ?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		return true;
	}
}
