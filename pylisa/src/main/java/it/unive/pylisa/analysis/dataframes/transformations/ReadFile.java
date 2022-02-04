package it.unive.pylisa.analysis.dataframes.transformations;

import it.unive.lisa.analysis.Lattice;

public class ReadFile extends DataframeTransformation {

	private final String file;

	public ReadFile(String s) {
		this.file = s;
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
		ReadFile other = (ReadFile) obj;
		if (file == null) {
			if (other.file != null)
				return false;
		} else if (!file.equals(other.file))
			return false;
		return true;
	}

	@Override
	public String toString() {
		if (isBottom())
			return Lattice.BOTTOM_STRING;
		else if (isTop())
			return Lattice.TOP_STRING;
		else
			return "file:" + file;
	}
}
