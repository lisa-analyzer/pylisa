package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.program.cfg.CodeLocation;

public class RowAccess extends DataframeOperation {

	private final Interval rows;

	public RowAccess(CodeLocation where, Interval rows) {
		super(where);
		this.rows = rows;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		RowAccess o = (RowAccess) other;
		return rows.lessOrEqual(o.rows);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		RowAccess o = (RowAccess) other;
		return new RowAccess(loc(other), rows.lub(o.rows));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((rows == null) ? 0 : rows.hashCode());
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
		RowAccess other = (RowAccess) obj;
		if (rows == null) {
			if (other.rows != null)
				return false;
		} else if (!rows.equals(other.rows))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "access_rows" + rows;
	}
}
