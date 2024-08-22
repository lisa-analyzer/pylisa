package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.constants.ConstantPropagation;

public class Read extends DataframeOperation {

	private final ConstantPropagation file;

	public Read(
			CodeLocation where,
			int index,
			ConstantPropagation file) {
		super(where, index);
		this.file = file;
	}

	public ConstantPropagation getFile() {
		return file;
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Read o = (Read) other;
		return file.lessOrEqual(o.file);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Read o = (Read) other;
		return new Read(where, index, file.lub(o.file));
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		Read other = (Read) o;
		return file.compareTo(other.file);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((file == null) ? 0 : file.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Read other = (Read) obj;
		if (file == null) {
			if (other.file != null)
				return false;
		} else if (!file.equals(other.file))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "read(" + file + ")";
	}
}
