package it.unive.pylisa.analysis.dataframes.transformations;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;

public class TopTransformation extends DataframeTransformation {

	TopTransformation() {
		// this is just to limit the creation of instances of this class
	}

	@Override
	public String toString() {
		return Lattice.TOP_STRING;
	}

	@Override
	protected DataframeTransformation lubAux(DataframeTransformation other) throws SemanticException {
		return this;
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformation other) throws SemanticException {
		return other instanceof TopTransformation;
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