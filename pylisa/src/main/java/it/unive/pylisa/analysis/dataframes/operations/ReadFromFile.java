package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;

public class ReadFromFile extends DataframeOperation {

	/**
	 * null represents an unknown file
	 */
	private final String file;

	public ReadFromFile(CodeLocation where, int index, String file) {
		super(where, index);
		this.file = file;
	}

	public String getFile() {
		return file;
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
		return new ReadFromFile(where, index, file(other));
	}

	private String file(DataframeOperation other) {
		ReadFromFile o = (ReadFromFile) other;
		if (file == null)
			return null;
		else if (o.file == null)
			return null;
		else if (!file.equals(o.file))
			return null;
		else
			return file;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((file == null) ? 0 : file.hashCode());
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

	@Override
	protected int compareToSameOperation(DataframeOperation o) {
		ReadFromFile other = (ReadFromFile) o;
		if (file == null && other.file != null)
			return 1;
		else if (file != null && other.file == null)
			return -1;
		else if (file == null)
			return 0;
		else
			return file.compareTo(other.file);
	}

	@Override
	protected DataframeOperation wideningSameOperation(DataframeOperation other) throws SemanticException {
		return new ReadFromFile(where, index, file(other));
	}
}
