package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.Names;

public class ColWrite extends DataframeOperation {

	private final Names cols;

	public ColWrite(CodeLocation where, Names cols) {
		super(where);
		this.cols = cols;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		ColWrite o = (ColWrite) other;
		return cols.lessOrEqual(o.cols);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		ColWrite o = (ColWrite) other;
		return new ColWrite(loc(other), cols.lub(o.cols));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((cols == null) ? 0 : cols.hashCode());
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
		ColWrite other = (ColWrite) obj;
		if (cols == null) {
			if (other.cols != null)
				return false;
		} else if (!cols.equals(other.cols))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "write_cols" + cols;
	}
}
