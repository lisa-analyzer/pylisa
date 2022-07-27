package it.unive.pylisa.analysis.dataframes.graph;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Function;

import org.apache.commons.collections4.CollectionUtils;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.outputs.serializableGraph.SerializableEdge;
import it.unive.lisa.outputs.serializableGraph.SerializableGraph;
import it.unive.lisa.outputs.serializableGraph.SerializableNode;
import it.unive.lisa.outputs.serializableGraph.SerializableNodeDescription;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import it.unive.lisa.util.datastructures.graph.AdjacencyMatrix;
import it.unive.lisa.util.datastructures.graph.BaseGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class DataframeForest extends BaseGraph<DataframeForest, DataframeOperation, DataframeEdge>
		implements Lattice<DataframeForest> {

	private final boolean isTop;

	public DataframeForest(boolean isTop) {
		this.isTop = isTop;
	}

	public DataframeForest(Collection<DataframeOperation> entrypoints,
			AdjacencyMatrix<DataframeForest, DataframeOperation, DataframeEdge> adjacencyMatrix,
			boolean isTop) {
		super(entrypoints, adjacencyMatrix);
		this.isTop = isTop;
	}

	public DataframeForest(DataframeForest other) {
		super(other);
		this.isTop = other.isTop;
	}

	@Override
	public SerializableGraph toSerializableGraph(Function<DataframeOperation, SerializableValue> descriptionGenerator) {
		String name = "dataframes";

		SortedSet<SerializableNode> nodes = new TreeSet<>();
		Map<DataframeOperation, Integer> nodeIds = new HashMap<>();
		SortedSet<SerializableNodeDescription> descrs = new TreeSet<>();
		SortedSet<SerializableEdge> edges = new TreeSet<>();

		for (DataframeOperation node : getNodes()) {
			nodeIds.put(node, addNode(nodes, descrs, node, descriptionGenerator));
		}

		for (DataframeOperation src : getNodes())
			for (DataframeOperation dest : followersOf(src))
				for (DataframeEdge edge : adjacencyMatrix.getEdgesConnecting(src, dest))
					edges.add(new SerializableEdge(nodeIds.get(src), nodeIds.get(dest), edge.getClass().getSimpleName()));

		return new SerializableGraph(name, null, nodes, edges, descrs);
	}
	
	private static int counter = 0;
	
	private static int addNode(
			SortedSet<SerializableNode> nodes,
			SortedSet<SerializableNodeDescription> descrs,
			DataframeOperation node,
			Function<DataframeOperation, SerializableValue> descriptionGenerator) {
		SerializableNode n = new SerializableNode(counter, Collections.emptyList(), node.toString());
		nodes.add(n);
		if (descriptionGenerator != null) {
			SerializableValue value = descriptionGenerator.apply(node);
			if (value != null)
				descrs.add(new SerializableNodeDescription(counter, value));
		}
		return counter++;
	}

	@Override
	public DataframeForest lub(DataframeForest other) throws SemanticException {
		if (other == null || other.isBottom() || this.isTop() || this == other || this.equals(other))
			return this;

		if (this.isBottom() || other.isTop())
			return other;

		DataframeForest forest = new DataframeForest(this);
		other.getNodes().forEach(forest::addNode);
		other.getEdges().forEach(forest::addEdge);
		return forest;
	}

	@Override
	public DataframeForest widening(DataframeForest other) throws SemanticException {
		return lub(other); // TODO only temporary
	}

	@Override
	public boolean lessOrEqual(DataframeForest other) throws SemanticException {
		if (other == null)
			return false;

		if (this == other || this.isBottom() || other.isTop() || this.equals(other))
			return true;

		if (this.isTop() || other.isBottom())
			return false;

		if (!CollectionUtils.isSubCollection(getNodes(), other.getNodes()))
			return false;

		return CollectionUtils.isSubCollection(getEdges(), other.getEdges());
	}

	@Override
	public DataframeForest top() {
		return new DataframeForest(true);
	}

	@Override
	public boolean isTop() {
		return adjacencyMatrix.getNodes().isEmpty() && isTop;
	}

	@Override
	public DataframeForest bottom() {
		return new DataframeForest(false);
	}

	@Override
	public boolean isBottom() {
		return adjacencyMatrix.getNodes().isEmpty() && !isTop;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;

		DataframeForest graph = (DataframeForest) obj;
		if (isTop != graph.isTop)
			return false;

		return super.isEqualTo(graph);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (isTop ? 1231 : 1237);
		result = prime * result + ((adjacencyMatrix == null) ? 0 : adjacencyMatrix.hashCode());
		result = prime * result + ((entrypoints == null) ? 0 : entrypoints.hashCode());
		return result;
	}

	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		return new StringRepresentation(adjacencyMatrix);
	}
}
