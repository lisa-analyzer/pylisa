package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.ColumnListSelection;
import java.util.HashSet;

public class DropColumns extends DataframeOperation {
	private ColumnListSelection columns;

	public DropColumns(CodeLocation where, ColumnListSelection columns) {
		super(where);
		this.columns = columns;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		DropColumns o = (DropColumns) other;
		if (this.columns == null)
			return o.columns == null;
		else if (o.columns == null)
			return false;
		return columns.lessOrEqual(o.columns);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		DropColumns o = (DropColumns) other;
		ColumnListSelection newColumns = new ColumnListSelection(new HashSet<>());
		if (this.columns != null)
			newColumns = newColumns.lub(this.columns);
		if (o.columns != null)
			newColumns = newColumns.lub(o.columns);
		return new DropColumns(loc(other), newColumns);
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DropColumns other = (DropColumns) obj;
		if (columns == null) {
			if (other.columns != null)
				return false;
		} else if (!columns.equals(other.columns))
			return false;
		return true;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((columns == null) ? 0 : columns.hashCode());
		return result;
	}

	@Override
	public String toString() {
		return "drop_columns(" + columns.toString() + ")";
	}

	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		DropColumns other = (DropColumns) o;
		return columns.compareTo(other.columns);
	}
}
