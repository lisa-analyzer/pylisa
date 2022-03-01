package it.unive.pylisa.analysis.dataframes.transformation.graph;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

import org.apache.commons.lang3.StringUtils;

import it.unive.lisa.util.collections.workset.LIFOWorkingSet;
import it.unive.lisa.util.collections.workset.VisitOnceWorkingSet;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ColAccess;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ColWrite;
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

			res.append(entry.getKey()).append(" -> [");

			Set<String> outs = new TreeSet<>();
			for (SimpleEdge out : entry.getValue().getOutgoing())
				outs.add(out.getDestination().toString());

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
		Set<SimpleEdge> union = new HashSet<>(edges.ingoing);
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

	public void addEdge(SimpleEdge e) {
		if (!matrix.containsKey(e.getSource()))
			throw new UnsupportedOperationException("The source node is not in the graph");

		if (!matrix.containsKey(e.getDestination()))
			throw new UnsupportedOperationException("The destination node is not in the graph");

		matrix.get(e.getSource()).outgoing.add(e);
		matrix.get(e.getDestination()).ingoing.add(e);
	}

	public void removeEdge(SimpleEdge e) {
		if (!matrix.containsKey(e.getSource()) || !matrix.containsKey(e.getDestination()))
			return;

		matrix.get(e.getSource()).outgoing.remove(e);
		matrix.get(e.getDestination()).ingoing.remove(e);
	}

	public final Collection<SimpleEdge> getIngoingEdges(DataframeOperation node) {
		return matrix.get(node).ingoing;
	}

	public final Collection<SimpleEdge> getOutgoingEdges(DataframeOperation node) {
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

		return matrix.get(node).outgoing.stream().map(SimpleEdge::getDestination).collect(Collectors.toSet());
	}

	public final Collection<DataframeOperation> predecessorsOf(DataframeOperation node) {
		if (!matrix.containsKey(node))
			throw new IllegalArgumentException("'" + node + "' is not in the graph");

		return matrix.get(node).ingoing.stream().map(SimpleEdge::getSource).collect(Collectors.toSet());
	}

	public boolean containsNode(DataframeOperation node) {
		return matrix.containsKey(node);
	}

	public boolean containsEdge(SimpleEdge edge) {
		for (Edges edges : matrix.values())
			for (SimpleEdge e : edges.outgoing)
				if (e == edge || e.equals(edge))
					return true;

		return false;
	}

	public static class Edges {
		private final Set<SimpleEdge> ingoing;
		private final Set<SimpleEdge> outgoing;

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
		public Set<SimpleEdge> getIngoing() {
			return ingoing;
		}

		/**
		 * Yields the outgoing edges.
		 * 
		 * @return the set of outgoing edges
		 */
		public Set<SimpleEdge> getOutgoing() {
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

	public void changeLastAccessToWrite() {
		VisitOnceWorkingSet<DataframeOperation> ws = VisitOnceWorkingSet.mk(LIFOWorkingSet.mk());
		getExits().forEach(ws::push);

		while (!ws.isEmpty()) {
			DataframeOperation current = ws.pop();
			Collection<DataframeOperation> preds = predecessorsOf(current);
			if (current instanceof ColAccess) {
				Collection<DataframeOperation> follows = followersOf(current);
				removeNode(current);
				ColWrite write = new ColWrite(current.getWhere(), ((ColAccess) current).getCols());
				addNode(write);
				preds.forEach(p -> addEdge(new SimpleEdge(p, write)));
				follows.forEach(f -> addEdge(new SimpleEdge(write, f)));
			} else
				preds.forEach(ws::push);
		}
	}
}
