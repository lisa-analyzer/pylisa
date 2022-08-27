package it.unive.pylisa.analysis.dataframes.edge;

import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class SimpleEdge extends DataframeEdge {

	public SimpleEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}

	@Override
	public String getEdgeSymbol() {
		return "->";
	}

	@Override
	public DataframeEdge newInstance(DataframeOperation source, DataframeOperation destination) {
		return new SimpleEdge(source, destination);
	}

	@Override
	public boolean isUnconditional() {
		return true;
	}
}
