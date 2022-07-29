package it.unive.pylisa.analysis.dataframes.transformation.graph;

import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class ConsumeEdge extends DataframeEdge {

	public ConsumeEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}

	@Override
	public String getEdgeSymbol() {
		return "-C->";
	}
	
	@Override
	public DataframeEdge mk(DataframeOperation source, DataframeOperation destination) {
		return new ConsumeEdge(source, destination);
	}
}