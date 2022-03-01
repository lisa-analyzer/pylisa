package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.program.cfg.CodeLocation;

public class FilterNullRows extends DataframeOperation {

	public FilterNullRows(CodeLocation where) {
		super(where);
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		return lessOrEqualSameOperation(other)
				? (where.equals(other.where) ? this : new FilterNullRows(loc(other)))
				: top();
	}

	@Override
	public int hashCode() {
		return super.hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "filter_null_rows()";
	}
}
