package it.unive.pylisa.analysis.dataframes.transformation.transformations;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;

public class BottomTransformation extends DataframeTransformation {

	BottomTransformation() {
		// this is just to limit the creation of instances of this class
	}

	@Override
	public String toString() {
		return Lattice.BOTTOM_STRING;
	}

	@Override
	protected DataframeTransformation lubAux(DataframeTransformation other) throws SemanticException {
		return other;
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformation other) throws SemanticException {
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
}