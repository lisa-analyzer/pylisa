package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.program.cfg.CodeLocation;

public class Keys extends DataframeOperation {

	public Keys(
			CodeLocation where) {
		super(where);
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other) {
		return new Keys(loc(other));
	}

	@Override
	public String toString() {
		return "get_keys";
	}

	@Override
	protected int compareToSameClassAndLocation(
			DataframeOperation o) {
		return 0;
	}
}
