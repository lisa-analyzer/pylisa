package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.Lattice;

public class ReadFromFile extends DataframeOperation {

	/**
	 * null represents an unknown file
	 */
	private final String file;

	public ReadFromFile(String file) {
		this.file = file;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		ReadFromFile o = (ReadFromFile) other;
		if (file == null)
			return o.file == null;
		else if (o.file == null)
			return true;
		else
			return file.equals(o.file);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		return lessOrEqualSameOperation(other) ? other : top();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((file == null) ? 0 : file.hashCode());
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
		ReadFromFile other = (ReadFromFile) obj;
		if (file == null) {
			if (other.file != null)
				return false;
		} else if (!file.equals(other.file))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "read(" + (file == null ? Lattice.TOP_STRING : file) + ")";
	}
}
