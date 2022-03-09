package it.unive.pylisa.analysis.dataframes.transformation.graph;

import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class AssignEdge extends DataframeEdge {

	public AssignEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}

	@Override
	public String toString() {
		return getSource() + " -A-> " + getDestination();
	}
}