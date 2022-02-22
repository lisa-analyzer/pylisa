package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.Lattice;

public class FilterNullRows extends DataframeOperation {

	public FilterNullRows() {
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		return lessOrEqualSameOperation(other) ? other : top();
	}

	@Override
	public int hashCode() {
		return this.getClass().hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
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
