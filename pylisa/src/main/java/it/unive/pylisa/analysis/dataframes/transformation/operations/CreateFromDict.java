package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.Names;

public class CreateFromDict extends DataframeOperation {

	private final Names knownColumns;

	public CreateFromDict(CodeLocation where, Names knownColumns) {
		super(where);
		this.knownColumns = knownColumns;
	}
	
	public Names getKnownColumns() {
		return knownColumns;
	}
	
	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		CreateFromDict o = (CreateFromDict) other;
		return knownColumns.lessOrEqual(o.knownColumns);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		CreateFromDict o = (CreateFromDict) other;
		return new CreateFromDict(loc(other), knownColumns.lub(o.knownColumns));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((knownColumns == null) ? 0 : knownColumns.hashCode());
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
		CreateFromDict other = (CreateFromDict) obj;
		if (knownColumns == null) {
			if (other.knownColumns != null)
				return false;
		} else if (!knownColumns.equals(other.knownColumns))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "fromDict(" + knownColumns + ")";
	}
	
	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		CreateFromDict other = (CreateFromDict) o;
		return knownColumns.compareTo(other.knownColumns);
	}
}
