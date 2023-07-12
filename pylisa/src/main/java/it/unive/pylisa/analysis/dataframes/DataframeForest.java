package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.outputs.DotGraph;
import it.unive.lisa.outputs.serializableGraph.SerializableEdge;
import it.unive.lisa.outputs.serializableGraph.SerializableGraph;
import it.unive.lisa.outputs.serializableGraph.SerializableNode;
import it.unive.lisa.outputs.serializableGraph.SerializableNodeDescription;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import it.unive.lisa.util.collections.workset.VisitOnceLIFOWorkingSet;
import it.unive.lisa.util.collections.workset.VisitOnceWorkingSet;
import it.unive.lisa.util.datastructures.graph.code.CodeGraph;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.analysis.dataframes.edge.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.edge.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.function.BiFunction;
import java.util.function.Predicate;
import org.apache.commons.collections4.CollectionUtils;
import org.apache.commons.lang3.StringUtils;
import org.graphstream.graph.Edge;
import org.graphstream.graph.Element;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.stream.file.FileSinkDOT;

public class DataframeForest
		extends CodeGraph<DataframeForest, DataframeOperation, DataframeEdge>
		implements Lattice<DataframeForest> {

	private final boolean isTop;

	public DataframeForest(boolean isTop) {
		super(new SimpleEdge(null, null));
		this.isTop = isTop;
	}

	public DataframeForest(Collection<DataframeOperation> entrypoints,
			NodeList<DataframeForest, DataframeOperation, DataframeEdge> adjacencyMatrix,
			boolean isTop) {
		super(entrypoints, adjacencyMatrix);
		this.isTop = isTop;
	}

	public DataframeForest(DataframeForest other) {
		super(other);
		this.isTop = other.isTop;
	}

	@Override
	public void addNode(DataframeOperation node, boolean entrypoint) {
		if (containsNode(node))
			for (DataframeOperation op : list)
				if (op.equals(node)) {
					node.setOffset(op.getOffset());
					return;
				}

		super.addNode(node, entrypoint);
	}

	@Override
	public void addEdge(DataframeEdge edge) {
		if (edge.getSource().equals(edge.getDestination()))
			// no self loops
			return;

		// we only keep 1 simple edge maximum, and only if there are no other
		// edges
		Collection<DataframeEdge> existing = getEdgesConnecting(edge.getSource(), edge.getDestination());
		existing.stream().filter(SimpleEdge.class::isInstance).forEach(list::removeEdge);

		super.addEdge(edge);
	}

	public void replace(DataframeOperation origin, DataframeOperation target) {
		addNode(target);
		for (DataframeEdge in : getIngoingEdges(origin))
			addEdge(in.newInstance(in.getSource(), target));
		for (DataframeEdge out : getOutgoingEdges(origin))
			addEdge(out.newInstance(target, out.getDestination()));
		list.removeNode(origin);
	}

	@Override
	public SerializableGraph toSerializableGraph(
			BiFunction<DataframeForest, DataframeOperation, SerializableValue> descriptionGenerator) {
		String name = "dataframes";

		SortedSet<SerializableNode> nodes = new TreeSet<>();
		Map<DataframeOperation, Integer> nodeIds = new HashMap<>();
		SortedSet<SerializableNodeDescription> descrs = new TreeSet<>();
		SortedSet<SerializableEdge> edges = new TreeSet<>();

		int counter = 0;
		for (DataframeOperation node : getNodes()) {
			addNode(counter, nodes, descrs, node, descriptionGenerator);
			nodeIds.put(node, counter);
			counter++;
		}

		for (DataframeOperation src : getNodes())
			for (DataframeOperation dest : followersOf(src))
				for (DataframeEdge edge : list.getEdgesConnecting(src, dest))
					if (edge instanceof ConcatEdge)
						edges.add(new ConcatSerializableEdge(nodeIds.get(src), nodeIds.get(dest),
								edge.getClass().getSimpleName(), ((ConcatEdge) edge).getEdgeIndex()));
					else
						edges.add(new SerializableEdge(nodeIds.get(src), nodeIds.get(dest),
								edge.getClass().getSimpleName()));

		return new CustomSerializableGraph(name, null, nodes, edges, descrs);
	}

	private void addNode(
			int id,
			SortedSet<SerializableNode> nodes,
			SortedSet<SerializableNodeDescription> descrs,
			DataframeOperation node,
			BiFunction<DataframeForest, DataframeOperation, SerializableValue> descriptionGenerator) {
		SerializableNode n = new SerializableNode(id, Collections.emptyList(), node.toString());
		nodes.add(n);
		if (descriptionGenerator != null) {
			SerializableValue value = descriptionGenerator.apply(this, node);
			if (value != null)
				descrs.add(new SerializableNodeDescription(id, value));
		}
	}

	@Override
	public DataframeForest lub(DataframeForest other) throws SemanticException {
		if (other == null || other.isBottom() || this.isTop() || this == other || this.equals(other))
			return this;

		if (this.isBottom() || other.isTop())
			return other;

		DataframeForest forest = union(other);
		return forest;
	}

	public DataframeForest union(DataframeForest other) {
		NodeList<DataframeForest, DataframeOperation, DataframeEdge> res = new NodeList<>(this.list);
		res.mergeWith(other.list);
		DataframeForest forest = new DataframeForest(Collections.emptySet(), res, false);
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
		return list.getNodes().isEmpty() && isTop;
	}

	@Override
	public DataframeForest bottom() {
		return new DataframeForest(false);
	}

	@Override
	public boolean isBottom() {
		return list.getNodes().isEmpty() && !isTop;
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
		result = prime * result + ((list == null) ? 0 : list.hashCode());
		result = prime * result + ((entrypoints == null) ? 0 : entrypoints.hashCode());
		return result;
	}

	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		return new StringRepresentation(deterministicToString());
	}

	private String deterministicToString() {
		StringBuilder res = new StringBuilder();

		SortedMap<DataframeOperation, Set<String>> aux = new TreeMap<>();
		for (DataframeOperation entry : list) {
			Set<String> outs = new TreeSet<>();
			for (DataframeEdge out : list.getOutgoingEdges(entry))
				outs.add(out.getEdgeSymbol() + " " + out.getDestination().toString());

			if (list.getIngoingEdges(entry).isEmpty())
				aux.put(entry, outs);
			else
				aux.put(entry, outs);
		}

		Collection<DataframeOperation> entries = list.getEntries();
		for (Entry<DataframeOperation, Set<String>> entry : aux.entrySet()) {
			if (entries.contains(entry.getKey()))
				res.append("*");
			res.append(entry.getKey()).append(": [");
			res.append(StringUtils.join(entry.getValue(), ", "));
			res.append("]\n");
		}

		return res.toString().trim();
	}

	static class ConcatSerializableEdge extends SerializableEdge {
		private final int index;

		public ConcatSerializableEdge(int sourceId, int destId, String kind, int index) {
			super(sourceId, destId, kind);
			this.index = index;
		}

		public int getIndex() {
			return index;
		}
	}

	static class CustomSerializableGraph extends SerializableGraph {

		public CustomSerializableGraph(String name, String description, SortedSet<SerializableNode> nodes,
				SortedSet<SerializableEdge> edges, SortedSet<SerializableNodeDescription> descriptions) {
			super(name, description, nodes, edges, descriptions);
		}

		@Override
		public DotGraph toDot() {
			DotGraph graph = new CustomDotGraph(getName());

			Set<Integer> hasFollows = new HashSet<>();
			Set<Integer> hasPreds = new HashSet<>();
			Set<Integer> inners = new HashSet<>();
			Map<Integer, SerializableValue> labels = new HashMap<>();

			getEdges().forEach(e -> {
				hasFollows.add(e.getSourceId());
				hasPreds.add(e.getDestId());
			});

			getDescriptions().forEach(d -> labels.put(d.getNodeId(), d.getDescription()));
			getNodes().forEach(n -> inners.addAll(n.getSubNodes()));

			for (SerializableNode n : getNodes())
				if (!inners.contains(n.getId()))
					graph.addNode(n, !hasPreds.contains(n.getId()), !hasFollows.contains(n.getId()),
							labels.get(n.getId()));

			for (SerializableEdge e : getEdges())
				graph.addEdge(e);

			return graph;
		}
	}

	static class CustomDotGraph extends DotGraph {

		public CustomDotGraph(String title) {
			super(title);
		}

		@Override
		public void addEdge(SerializableEdge edge) {
			long id = edge.getSourceId();
			long id1 = edge.getDestId();

			Edge e = graph.addEdge(edgeName(id, id1, edge), nodeName(id), nodeName(id1), true);

			switch (edge.getKind()) {
			case "ConcatEdge":
				e.setAttribute(COLOR, COLOR_RED);
				e.setAttribute(LABEL, ((ConcatSerializableEdge) edge).getIndex());
				break;
			case "AssignEdge":
				e.setAttribute(COLOR, COLOR_BLUE);
				e.setAttribute(LABEL, "A");
				break;
			case "ConsumeEdge":
				e.setAttribute(STYLE, CONDITIONAL_EDGE_STYLE);
				e.setAttribute(COLOR, COLOR_BLACK);
				break;
			case "SimpleEdge":
			default:
				e.setAttribute(COLOR, COLOR_BLACK);
				break;
			}
		}

		protected static String edgeName(long src, long dest, SerializableEdge edge) {
			return "edge-" + src + "-" + dest + "-" + edge.getKind();
		}

		private class CustomDotSink extends FileSinkDOT {

			@Override
			protected void outputHeader() throws IOException {
				out = (PrintWriter) output;
				out.printf("%s {%n", "digraph");
			}

			@Override
			protected String outputAttribute(String key, Object value, boolean first) {
				boolean quote = true;

				if (value instanceof Number || key.equals(LABEL))
					// labels that we output are always in html format
					// so no need to quote them
					quote = false;

				Object quoting = quote ? "\"" : "";
				return String.format("%s%s=%s%s%s", first ? "" : ",", key, quoting, value, quoting);
			}

			@Override
			protected String outputAttributes(Element e) {
				if (e.getAttributeCount() == 0)
					return "";

				Map<String, String> attrs = new HashMap<>();
				e.attributeKeys().forEach(key -> attrs.put(key, outputAttribute(key, e.getAttribute(key), true)));

				StringBuilder buffer = new StringBuilder("[");
				for (Entry<String, String> entry : attrs.entrySet())
					if (!entry.getKey().equals(LABEL))
						buffer.append(entry.getValue()).append(",");

				if (attrs.containsKey(LABEL))
					buffer.append(attrs.get(LABEL));

				String result = buffer.toString();
				if (result.endsWith(","))
					result = result.substring(0, result.length() - 1);

				return result + "]";
			}
		}

		@Override
		public void dump(Writer writer) throws IOException {
			FileSinkDOT sink = new CustomDotSink() {
				@Override
				protected void outputEndOfFile() throws IOException {
					LegendClusterSink legend = new LegendClusterSink();
					legend.setDirected(true);
					StringWriter sw = new StringWriter();
					legend.writeAll(new Legend().graph, sw);
					out.printf("%s%n", sw.toString());
					super.outputEndOfFile();
				}
			};
			sink.setDirected(true);
			sink.writeAll(graph, writer);
		}

		private class LegendClusterSink extends CustomDotSink {
			@Override
			protected void outputHeader() throws IOException {
				out = (PrintWriter) output;
				out.printf("%s {%n", "subgraph cluster_legend");
				out.printf("\tlabel=\"Legend\";%n");
				out.printf("\tstyle=dotted;%n");
				out.printf("\tnode [shape=plaintext];%n");
			}
		}

		private static final class Legend {
			private final org.graphstream.graph.Graph graph;

			private Legend() {
				graph = new MultiGraph("legend");
				org.graphstream.graph.Node l = graph.addNode("legend");
				StringBuilder builder = new StringBuilder();
				builder.append("<");
				builder.append("<table border=\"0\" cellpadding=\"2\" cellspacing=\"0\" cellborder=\"0\">");
				builder.append("<tr><td align=\"right\">node border&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(NORMAL_NODE_COLOR);
				builder.append("\">");
				builder.append(NORMAL_NODE_COLOR);
				builder.append("</font>, single</td></tr>");
				builder.append("<tr><td align=\"right\">entrypoint border&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(SPECIAL_NODE_COLOR);
				builder.append("\">");
				builder.append(SPECIAL_NODE_COLOR);
				builder.append("</font>, single</td></tr>");
				builder.append("<tr><td align=\"right\">exitpoint border&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(SPECIAL_NODE_COLOR);
				builder.append("\">");
				builder.append(SPECIAL_NODE_COLOR);
				builder.append("</font>, double</td></tr>");
				builder.append("<tr><td align=\"right\">sequential edge&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(COLOR_BLACK);
				builder.append("\">");
				builder.append(COLOR_BLACK);
				builder.append("</font>, solid</td></tr>");
				builder.append("<tr><td align=\"right\">assign edge&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(COLOR_BLUE);
				builder.append("\">");
				builder.append(COLOR_BLUE);
				builder.append("</font>, solid</td></tr>");
				builder.append("<tr><td align=\"right\">concat edge&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(COLOR_RED);
				builder.append("\">");
				builder.append(COLOR_RED);
				builder.append("</font>, solid</td></tr>");
				builder.append("<tr><td align=\"right\">consume edge&nbsp;</td><td align=\"left\"><font color=\"");
				builder.append(COLOR_BLACK);
				builder.append("\">");
				builder.append(COLOR_BLACK);
				builder.append("</font>, ");
				builder.append(CONDITIONAL_EDGE_STYLE);
				builder.append("</td></tr>");
				builder.append("</table>");
				builder.append(">");
				l.setAttribute("label", builder.toString());
			}
		}
	}

	public Collection<DataframeForest> partitionByRoot() {
		Collection<DataframeOperation> entries = list.getEntries();
		Collection<DataframeForest> result = new ArrayList<>(entries.size());
		for (DataframeOperation entry : entries)
			result.add(dfs(entry));
		return result;
	}

	private DataframeForest dfs(DataframeOperation entry) {
		NodeList<DataframeForest, DataframeOperation,
				DataframeEdge> list = new NodeList<>(new SimpleEdge(null, null), false);
		DataframeForest forest = new DataframeForest(Collections.singleton(entry), list, false);
		VisitOnceWorkingSet<DataframeOperation> ws = VisitOnceLIFOWorkingSet.mk();
		Set<DataframeEdge> seenEdges = new TreeSet<>();
		ws.push(entry);
		list.addNode(entry);

		while (!ws.isEmpty()) {
			DataframeOperation current = ws.pop();
			for (DataframeEdge edge : getOutgoingEdges(current)) {
				if (!ws.getSeen().contains(edge.getDestination())) {
					list.addNode(edge.getDestination());
					ws.push(edge.getDestination());
				}
				if (!seenEdges.contains(edge)) {
					list.addEdge(edge);
					seenEdges.add(edge);
				}
			}
		}

		return forest;
	}

	public DataframeForest bDFS(DataframeOperation leaf,
			Predicate<DataframeOperation> stop,
			Predicate<DataframeEdge> followEdge) {
		NodeList<DataframeForest, DataframeOperation,
				DataframeEdge> list = new NodeList<>(new SimpleEdge(null, null), false);
		DataframeForest forest = new DataframeForest(Collections.emptySet(), list, false);
		VisitOnceWorkingSet<DataframeOperation> ws = VisitOnceLIFOWorkingSet.mk();
		Set<DataframeEdge> seenEdges = new TreeSet<>();
		ws.push(leaf);
		list.addNode(leaf);

		while (!ws.isEmpty()) {
			DataframeOperation current = ws.pop();
			if (stop.test(current))
				continue;
			for (DataframeEdge edge : getIngoingEdges(current)) {
				if (!followEdge.test(edge))
					continue;
				if (!ws.getSeen().contains(edge.getSource())) {
					list.addNode(edge.getSource());
					ws.push(edge.getSource());
				}
				if (!seenEdges.contains(edge)) {
					list.addEdge(edge);
					seenEdges.add(edge);
				}
			}
		}

		return forest;
	}
}
