package it.unive.pylisa.analysis.dataframes.edge;

import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class ConsumeEdge extends DataframeEdge {

	public ConsumeEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}

	@Override
	public String getEdgeSymbol() {
		return "-C->";
	}

	@Override
	public DataframeEdge newInstance(DataframeOperation source, DataframeOperation destination) {
		return new ConsumeEdge(source, destination);
	}
}