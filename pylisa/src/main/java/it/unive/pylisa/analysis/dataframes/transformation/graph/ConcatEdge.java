package it.unive.pylisa.analysis.dataframes.transformation.graph;

import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class ConcatEdge extends DataframeEdge {

	private final int edgeIndex;

	public ConcatEdge(DataframeOperation source, DataframeOperation destination, int edgeIndex) {
		super(source, destination);
		this.edgeIndex = edgeIndex;
	}

	public int getEdgeIndex() {
		return edgeIndex;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + edgeIndex;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		ConcatEdge other = (ConcatEdge) obj;
		if (edgeIndex != other.edgeIndex)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return getSource() + " -" + edgeIndex + "-> " + getDestination();
	}
}
