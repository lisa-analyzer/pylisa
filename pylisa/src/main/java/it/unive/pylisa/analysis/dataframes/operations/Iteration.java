package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.program.cfg.CodeLocation;

public class Iteration extends DataframeOperation {

	public Iteration(
			CodeLocation where,
			int index) {
		super(where, index);
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other) {
		return this;
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		return 0;
	}

	@Override
	public String toString() {
		return "iterate";
	}
}
