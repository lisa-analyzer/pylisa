package it.unive.pylisa.analysis.dataframes.transformation;

import java.io.IOException;
import java.io.Writer;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;

import org.apache.commons.text.StringEscapeUtils;
import org.graphstream.graph.implementations.MultiGraph;
import org.graphstream.stream.file.FileSinkDOT;

import it.unive.pylisa.analysis.dataframes.transformation.graph.AssignEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class DotDFGraph {

	private static final String COLOR_BLACK = "black";

	private static final String COLOR_GRAY = "gray";

	private static final String COLOR_RED = "red";

	private static final String COLOR_BLUE = "blue";

	private static final String STYLE = "style";

	private static final String COLOR = "color";

	private static final String SHAPE = "shape";

	private static final String LABEL = "label";

	private static final String EXIT_NODE_EXTRA_ATTR = "peripheries";

	private static final String NODE_SHAPE = "rect";

	private static final String EXIT_NODE_EXTRA_VALUE = "2";

	private static final String SPECIAL_NODE_COLOR = COLOR_BLACK;

	private static final String NORMAL_NODE_COLOR = COLOR_GRAY;

	// private static final String CONDITIONAL_EDGE_STYLE = "dashed";

	private final org.graphstream.graph.Graph graph;

	private final Map<DataframeOperation, Long> codes = new IdentityHashMap<>();

	private long nextCode;

	public DotDFGraph(DataframeGraph graph) {
		this.graph = new MultiGraph("graph");

		Collection<DataframeOperation> entries = graph.getEntries();
		Collection<DataframeOperation> exits = graph.getExits();

		for (DataframeOperation node : graph.getNodes())
			addNode(node, entries.contains(node), exits.contains(node));

		for (DataframeOperation src : graph.getNodes())
			for (DataframeOperation dest : graph.followersOf(src)) {
				DataframeEdge edge = graph.getEdgeConnecting(src, dest);
				if (edge instanceof AssignEdge)
					addEdge(edge, COLOR_BLUE, null);
				else if (edge instanceof ConcatEdge)
					addEdge(edge, COLOR_RED, null);
				else if (edge instanceof SimpleEdge)
					addEdge(edge, COLOR_BLACK, null);
				else
					throw new UnsupportedOperationException("Unsupported edge type " + edge.getClass().getName());
			}
	}

	private static String dotEscape(String extraLabel) {
		String escapeHtml4 = StringEscapeUtils.escapeHtml4(extraLabel);
		String replace = escapeHtml4.replace("\n", "<BR/>");
		return replace.replace("\\", "\\\\");
	}

	private void addNode(DataframeOperation node, boolean entry, boolean exit) {
		org.graphstream.graph.Node n = graph.addNode(nodeName(codes.computeIfAbsent(node, nn -> nextCode++)));

		n.setAttribute(SHAPE, NODE_SHAPE);
		if (entry || exit)
			n.setAttribute(COLOR, SPECIAL_NODE_COLOR);
		else
			n.setAttribute(COLOR, NORMAL_NODE_COLOR);

		if (exit)
			n.setAttribute(EXIT_NODE_EXTRA_ATTR, EXIT_NODE_EXTRA_VALUE);

		String label = dotEscape(node.toString());
		n.setAttribute(LABEL, label);
	}

	private static String nodeName(long id) {
		return "node" + id;
	}

	private void addEdge(DataframeEdge edge, String color, String style) {
		long id = codes.computeIfAbsent(edge.getSource(), n -> nextCode++);
		long id1 = codes.computeIfAbsent(edge.getDestination(), n -> nextCode++);

		org.graphstream.graph.Edge e = graph.addEdge("edge-" + id + "-" + id1, nodeName(id), nodeName(id1), true);

		if (style != null)
			e.setAttribute(STYLE, style);

		if (color != null)
			e.setAttribute(COLOR, color);
		
		if (edge instanceof AssignEdge)
			e.setAttribute(LABEL, "A");
		else if (edge instanceof ConcatEdge)
			e.setAttribute(LABEL, ((ConcatEdge) edge).getEdgeIndex());
	}

	public void dumpDot(Writer writer) throws IOException {
		FileSinkDOT sink = new FileSinkDOT();
		sink.setDirected(true);
		sink.writeAll(graph, writer);
	}
}
