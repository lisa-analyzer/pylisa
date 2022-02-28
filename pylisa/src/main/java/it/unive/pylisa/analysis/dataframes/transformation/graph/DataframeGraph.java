package it.unive.pylisa.analysis.dataframes.transformation.graph;

import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Function;

import org.apache.commons.lang3.StringUtils;

import it.unive.lisa.outputs.DotGraph;
import it.unive.lisa.util.datastructures.graph.Graph;
import it.unive.lisa.util.datastructures.graph.AdjacencyMatrix.NodeEdges;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class DataframeGraph extends Graph<DataframeGraph, DataframeOperation, SimpleEdge> {

	public DataframeGraph() {
		super();
	}

	public DataframeGraph(DataframeGraph other) {
		super(other);
	}

	@Override
	protected DotGraph<DataframeOperation, SimpleEdge, DataframeGraph> toDot(
			Function<DataframeOperation, String> labelGenerator) {
		throw new UnsupportedOperationException();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((adjacencyMatrix == null) ? 0 : adjacencyMatrix.hashCode());
		result = prime * result + ((entrypoints == null) ? 0 : entrypoints.hashCode());
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
		DataframeGraph other = (DataframeGraph) obj;
		if (adjacencyMatrix == null) {
			if (other.adjacencyMatrix != null)
				return false;
		} else if (!adjacencyMatrix.equals(other.adjacencyMatrix))
			return false;
		if (entrypoints == null) {
			if (other.entrypoints != null)
				return false;
		} else if (!entrypoints.equals(other.entrypoints))
			return false;
		return true;
	}

	@Override
	public String toString() {
		StringBuilder res = new StringBuilder();
		for (Entry<DataframeOperation,
				NodeEdges<DataframeOperation, SimpleEdge, DataframeGraph>> entry : adjacencyMatrix) {
			if (entrypoints.contains(entry.getKey()))
				res.append("*");

			res.append(entry.getKey()).append(" -> [");

			Set<String> outs = new TreeSet<>();
			for (SimpleEdge out : entry.getValue().getOutgoing())
				outs.add(out.getDestination().toString());

			res.append(StringUtils.join(outs, ", "));
			res.append("]\n");
		}
		return res.toString().trim();
	}
}
