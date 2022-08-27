package it.unive.pylisa.analysis.dataframes.edge;

import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class AssignEdge extends DataframeEdge {

	public AssignEdge(DataframeOperation source, DataframeOperation destination) {
		super(source, destination);
	}

	@Override
	public String getEdgeSymbol() {
		return "-A->";
	}

	@Override
	public DataframeEdge newInstance(DataframeOperation source, DataframeOperation destination) {
		return new AssignEdge(source, destination);
	}
}