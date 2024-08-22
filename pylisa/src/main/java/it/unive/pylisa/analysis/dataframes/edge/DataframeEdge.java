package it.unive.pylisa.analysis.dataframes.edge;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.lisa.util.datastructures.graph.code.CodeEdge;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public abstract class DataframeEdge
		implements
		CodeEdge<DataframeForest, DataframeOperation, DataframeEdge> {

	private final DataframeOperation source, destination;

	protected DataframeEdge(
			DataframeOperation source,
			DataframeOperation destination) {
		this.source = source;
		this.destination = destination;
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
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((destination == null) ? 0 : destination.hashCode());
		result = prime * result + ((source == null) ? 0 : source.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DataframeEdge other = (DataframeEdge) obj;
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
		return true;
	}

	@Override
	public final String toString() {
		return source + " " + getEdgeSymbol() + " " + destination;
	}

	public abstract String getEdgeSymbol();

	@Override
	public <V> boolean accept(
			GraphVisitor<DataframeForest, DataframeOperation, DataframeEdge, V> visitor,
			V tool) {
		return visitor.visit(tool, null, this);
	}

	@Override
	public int compareTo(
			DataframeEdge o) {
		int cmp;
		if ((cmp = source.compareTo(o.source)) != 0)
			return cmp;
		if ((cmp = destination.compareTo(o.destination)) != 0)
			return cmp;
		return compareKind(o);
	}

	public int compareKind(
			DataframeEdge o) {
		int cmp;
		if ((cmp = getClass().getName().compareTo(o.getClass().getName())) != 0)
			return cmp;
		return compareToSameEdgeKind(o);
	}

	public boolean sameKind(
			DataframeEdge o) {
		return compareKind(o) == 0;
	}

	protected int compareToSameEdgeKind(
			DataframeEdge o) {
		return 0;
	}

	@Override
	public boolean isUnconditional() {
		return false;
	}

	public boolean lessOrEqual(
			DataframeEdge other)
			throws SemanticException {
		return getClass() == other.getClass()
				&& source.lessOrEqual(other.source)
				&& destination.lessOrEqual(other.destination);
	}
}
