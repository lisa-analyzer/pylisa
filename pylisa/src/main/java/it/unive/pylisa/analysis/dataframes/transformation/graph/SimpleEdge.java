package it.unive.pylisa.analysis.dataframes.transformation.graph;

import it.unive.lisa.util.datastructures.graph.Edge;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class SimpleEdge implements Edge<DataframeOperation, SimpleEdge, DataframeGraph> {

	private final DataframeOperation source, destination;

	// when we have a concatenation, the order of the concatenation matters so we add
	// an optional index to each edge
	private int edgeIndex;

	public SimpleEdge(DataframeOperation source, DataframeOperation destination) {
		this.source = source;
		this.destination = destination;
		this.edgeIndex = -1;
	}

	public SimpleEdge(DataframeOperation source, DataframeOperation destination, int edgeIndex) {
		this.source = source;
		this.destination = destination;
		this.edgeIndex = edgeIndex;
	}

	@Override
	public DataframeOperation getSource() {
		return source;
	}

	@Override
	public DataframeOperation getDestination() {
		return destination;
	}

	@Override
	public boolean canBeSimplified() {
		return false;
	}

	@Override
	public SimpleEdge newInstance(DataframeOperation source, DataframeOperation destination) {
		return new SimpleEdge(source, destination);
	}

	@Override
	public <V> boolean accept(GraphVisitor<DataframeGraph, DataframeOperation, SimpleEdge, V> visitor, V tool) {
		return visitor.visit(tool, null, this);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((destination == null) ? 0 : destination.hashCode());
		result = prime * result + ((source == null) ? 0 : source.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		SimpleEdge other = (SimpleEdge) obj;
		if (destination == null) {
			if (other.destination != null)
				return false;
		} else if (!destination.equals(other.destination))
			return false;
		if (source == null) {
			if (other.source != null)
				return false;
		} else if (!source.equals(other.source))
			return false;
		if (this.edgeIndex != other.edgeIndex)
			return false;
		return true;
	}

	@Override
	public String toString() {
		if (edgeIndex != -1)
			return source + " (" + Integer.toString(this.edgeIndex) + ")" + "-> " + destination;
		return source + " -> " + destination;
	}
}
