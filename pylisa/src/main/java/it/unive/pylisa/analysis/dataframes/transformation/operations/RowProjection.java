package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.program.cfg.CodeLocation;

public class RowProjection extends DataframeOperation {

	private final Interval rows;

	public RowProjection(CodeLocation where, Interval rows) {
		super(where);
		this.rows = rows;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		RowProjection o = (RowProjection) other;
		return rows.lessOrEqual(o.rows);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		RowProjection o = (RowProjection) other;
		return new RowProjection(loc(other), rows.lub(o.rows));
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
		RowProjection other = (RowProjection) obj;
		if (rows == null) {
			if (other.rows != null)
				return false;
		} else if (!rows.equals(other.rows))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "proj_rows" + rows;
	}
}
