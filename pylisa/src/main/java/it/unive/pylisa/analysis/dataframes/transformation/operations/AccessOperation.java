package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.Selection;

public class AccessOperation<S extends Selection<S>> extends SelectionOperation<S> {

	public AccessOperation(CodeLocation where, S selection) {
		super(where, selection);
	}

	@Override
	public String toString() {
		return "access:" + getSelection();
	}

	@Override
	public SelectionOperation<S> mk(CodeLocation where, S selection) {
		return new AccessOperation<>(where, selection);
	}
}
