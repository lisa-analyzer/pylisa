package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;

public abstract class Selection<S extends Selection<S>> extends BaseLattice<S> {

	@SuppressWarnings("unchecked")
	public S lub(Selection<?> other) throws SemanticException {
		return lub((S) other);
	}
}
