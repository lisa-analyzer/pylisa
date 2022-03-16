package it.unive.pylisa.analysis.dataframes.transformation.graph;

import java.io.IOException;
import java.io.Writer;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.StringUtils;

import it.unive.pylisa.analysis.dataframes.transformation.DotDFGraph;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class DataframeGraph {

	private final Map<DataframeOperation, Edges> matrix;

	public DataframeGraph() {
		matrix = new HashMap<>();
	}

	public DataframeGraph(DataframeGraph other) {
		this(other.matrix);
	}

	public DataframeGraph(Map<DataframeOperation, Edges> matrix) {
		this.matrix = new HashMap<>();
		for (Entry<DataframeOperation, Edges> entry : matrix.entrySet())
			this.matrix.put(entry.getKey(), new Edges(entry.getValue()));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((matrix == null) ? 0 : matrix.hashCode());
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
		if (matrix == null) {
			if (other.matrix != null)
				return false;
		} else if (!matrix.equals(other.matrix))
			return false;
		return true;
	}

	@Override
	public String toString() {
		StringBuilder res = new StringBuilder();
		for (Entry<DataframeOperation, Edges> entry : matrix.entrySet()) {
			if (entry.getValue().ingoing.isEmpty())
				res.append("*");

			res.append(entry.getKey()).append(": [");

			Set<String> outs = new TreeSet<>();
			for (DataframeEdge out : entry.getValue().getOutgoing())
				outs.add(out.getEdgeSymbol() + " " + out.getDestination().toString());

			res.append(StringUtils.join(outs, ", "));
			res.append("]\n");
		}
		return res.toString().trim();
	}

	public final void addNode(DataframeOperation node) {
		matrix.putIfAbsent(node, new Edges());
	}

	public void removeNode(DataframeOperation node) {
		if (!containsNode(node))
			return;

		Edges edges = matrix.get(node);
		Set<DataframeEdge> union = new HashSet<>(edges.ingoing);
		union.addAll(edges.outgoing);
		union.forEach(this::removeEdge);
		matrix.remove(node);
	}

	public final Collection<DataframeOperation> getNodes() {
		return matrix.keySet();
	}

	public final int getNodesCount() {
		return getNodes().size();
	}

	public void addEdge(DataframeEdge e) {
		if (!matrix.containsKey(e.getSource()))
			throw new UnsupportedOperationException("The source node is not in the graph");

		if (!matrix.containsKey(e.getDestination()))
			throw new UnsupportedOperationException("The destination node is not in the graph");

		matrix.get(e.getSource()).outgoing.add(e);
		matrix.get(e.getDestination()).ingoing.add(e);
	}

	public void removeEdge(DataframeEdge e) {
		if (!matrix.containsKey(e.getSource()) || !matrix.containsKey(e.getDestination()))
			return;

		matrix.get(e.getSource()).outgoing.remove(e);
		matrix.get(e.getDestination()).ingoing.remove(e);
	}

	public final Collection<DataframeEdge> getIngoingEdges(DataframeOperation node) {
		return matrix.get(node).ingoing;
	}

	public final Collection<DataframeEdge> getOutgoingEdges(DataframeOperation node) {
		return matrix.get(node).outgoing;
	}

	public Collection<DataframeOperation> getEntries() {
		return matrix.entrySet().stream().filter(e -> e.getValue().ingoing.isEmpty()).map(Entry::getKey)
				.collect(Collectors.toSet());
	}

	public Collection<DataframeOperation> getExits() {
		return matrix.entrySet().stream().filter(e -> e.getValue().outgoing.isEmpty()).map(Entry::getKey)
				.collect(Collectors.toSet());
	}

	public final Collection<DataframeOperation> followersOf(DataframeOperation node) {
		if (!matrix.containsKey(node))
			throw new IllegalArgumentException("'" + node + "' is not in the graph");

		return matrix.get(node).outgoing.stream().map(DataframeEdge::getDestination).collect(Collectors.toSet());
	}

	public final Collection<DataframeOperation> predecessorsOf(DataframeOperation node) {
		if (!matrix.containsKey(node))
			throw new IllegalArgumentException("'" + node + "' is not in the graph");

		return matrix.get(node).ingoing.stream().map(DataframeEdge::getSource).collect(Collectors.toSet());
	}

	public boolean containsNode(DataframeOperation node) {
		return matrix.containsKey(node);
	}

	public boolean containsEdge(SimpleEdge edge) {
		for (Edges edges : matrix.values())
			for (DataframeEdge e : edges.outgoing)
				if (e == edge || e.equals(edge))
					return true;

		return false;
	}

	public final Collection<DataframeEdge> getEdges() {
		return matrix.values().stream()
				.flatMap(c -> Stream.concat(c.ingoing.stream(), c.outgoing.stream()))
				.distinct()
				.collect(Collectors.toSet());
	}

	public static class Edges {
		private final Set<DataframeEdge> ingoing;
		private final Set<DataframeEdge> outgoing;

		private Edges() {
			ingoing = new HashSet<>();
			outgoing = new HashSet<>();
		}

		private Edges(Edges other) {
			ingoing = new HashSet<>(other.ingoing);
			outgoing = new HashSet<>(other.outgoing);
		}

		/**
		 * Yields the ingoing edges.
		 * 
		 * @return the set of ingoing edges
		 */
		public Set<DataframeEdge> getIngoing() {
			return ingoing;
		}

		/**
		 * Yields the outgoing edges.
		 * 
		 * @return the set of outgoing edges
		 */
		public Set<DataframeEdge> getOutgoing() {
			return outgoing;
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + ((ingoing == null) ? 0 : ingoing.hashCode());
			result = prime * result + ((outgoing == null) ? 0 : outgoing.hashCode());
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
			Edges other = (Edges) obj;
			if (ingoing == null) {
				if (other.ingoing != null)
					return false;
			} else if (!ingoing.equals(other.ingoing))
				return false;
			if (outgoing == null) {
				if (other.outgoing != null)
					return false;
			} else if (!outgoing.equals(other.outgoing))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return "ins: " + ingoing + ", outs: " + outgoing;
		}
	}

	public void mergeWith(DataframeGraph other) {
		for (DataframeOperation node : other.getNodes())
			addNode(node);

		for (DataframeEdge edge : other.getEdges())
			addEdge(edge);
	}

	public DataframeOperation getLeaf() {
		Collection<DataframeOperation> exits = getExits();
		if (exits.size() != 1)
			throw new IllegalStateException("Dataframe graphs should always have a unique exit point (leaf)");
		return exits.iterator().next();
	}

	public DataframeGraph prefix() {
		DataframeGraph copy = new DataframeGraph(this);
		copy.removeNode(copy.getLeaf());
		return copy;
	}

	public DataframeEdge getEdgeConnecting(DataframeOperation source, DataframeOperation destination) {
		if (!matrix.containsKey(source))
			return null;

		for (DataframeEdge e : matrix.get(source).outgoing)
			if (e.getDestination().equals(destination))
				return e;

		return null;
	}

	public void dump(Writer writer) throws IOException {
		new DotDFGraph(this).dumpDot(writer);
	}
}
