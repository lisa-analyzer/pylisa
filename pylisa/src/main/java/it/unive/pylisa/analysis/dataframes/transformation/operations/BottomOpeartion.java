package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;

public class BottomOpeartion extends DataframeOperation {

	BottomOpeartion() {
		// this is just to limit the creation of instances of this class
	}

	@Override
	public String toString() {
		return Lattice.BOTTOM_STRING;
	}

	@Override
	protected DataframeOperation lubAux(DataframeOperation other) throws SemanticException {
		return other;
	}

	@Override
	protected boolean lessOrEqualAux(DataframeOperation other) throws SemanticException {
		return true;
	}

	@Override
	public int hashCode() {
		return getClass().hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		return getClass() == obj.getClass();
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		return this;
	}
}