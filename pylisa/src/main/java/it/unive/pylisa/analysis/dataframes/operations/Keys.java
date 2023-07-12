package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;

public class Keys extends DataframeOperation {

	public Keys(CodeLocation where, int index) {
		super(where, index);
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
	public String toString() {
		return "get_keys";
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
