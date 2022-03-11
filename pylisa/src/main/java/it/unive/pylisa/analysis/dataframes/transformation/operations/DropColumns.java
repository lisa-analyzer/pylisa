package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.Names;

public class DropColumns extends DataframeOperation {

	private final Names columns;

	public DropColumns(CodeLocation where, Names columns) {
		super(where);
		this.columns = columns;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		DropColumns o = (DropColumns) other;
		return columns.lessOrEqual(o.columns);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		DropColumns o = (DropColumns) other;
		return new DropColumns(loc(other), columns.lub(o.columns));
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
		return "drop_columns(" + columns + ")";
	}

}
