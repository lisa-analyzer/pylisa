package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CodeLocation;

public class TopOperation extends DataframeOperation {

	public TopOperation() {
		super(SyntheticLocation.INSTANCE, -1);
	}

	public TopOperation(CodeLocation where) {
		super(where, -1);
	}

	@Override
	public String toString() {
		return Lattice.TOP_STRING;
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
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		return this;
	}

	@Override
	protected int compareToSameOperation(DataframeOperation o) {
		return 0;
	}

	@Override
	protected DataframeOperation wideningSameOperation(DataframeOperation other) throws SemanticException {
		return this;
	}
}