package it.unive.pylisa.analysis.dataframes.transformation.graph;

import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class SimpleEdge extends DataframeEdge {

	public SimpleEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}
	
	@Override
	public String getEdgeSymbol() {
		return "->";
	}
}
