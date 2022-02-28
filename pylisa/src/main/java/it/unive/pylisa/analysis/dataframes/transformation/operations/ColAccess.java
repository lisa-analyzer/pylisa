package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.transformation.Names;

public class ColAccess extends DataframeOperation {

	private final Names rows;

	public ColAccess(Names rows) {
		this.rows = rows;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		ColAccess o = (ColAccess) other;
		return rows.lessOrEqual(o.rows);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		ColAccess o = (ColAccess) other;
		return new ColAccess(rows.lub(o.rows));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((rows == null) ? 0 : rows.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ColAccess other = (ColAccess) obj;
		if (rows == null) {
			if (other.rows != null)
				return false;
		} else if (!rows.equals(other.rows))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "access_cols" + rows;
	}
}
