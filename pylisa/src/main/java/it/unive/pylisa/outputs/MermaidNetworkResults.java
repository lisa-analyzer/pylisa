package it.unive.pylisa.outputs;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.outputs.DotGraph;
import it.unive.lisa.outputs.OutputCFGDumper;
import it.unive.lisa.outputs.serializableGraph.SerializableGraph;
import it.unive.lisa.outputs.serializableGraph.SerializableNode;
import it.unive.lisa.outputs.serializableGraph.SerializableNodeDescription;
import it.unive.lisa.util.file.FileManager;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Scanner;
import java.util.Set;
import java.util.stream.Collectors;
import org.apache.commons.lang3.StringUtils;
import org.thymeleaf.TemplateEngine;
import org.thymeleaf.context.Context;
import org.thymeleaf.templatemode.TemplateMode;
import org.thymeleaf.templateresolver.StringTemplateResolver;

/**
 * An output that dumps each analyzed CFG as an HTML file, with the right-hand
 * panel replaced by an interactive Mermaid {@code graph TD} diagram of the
 * {@code Network State} tracked by {@code NetworkAwareAbstractDomain}.
 *
 * @param <A> the kind of {@link AbstractLattice} produced by the domain
 *                {@code D}
 * @param <D> the kind of {@link AbstractDomain} to run during the analysis
 */
public class MermaidNetworkResults<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		extends
		OutputCFGDumper<A, D> {

	private final boolean withSubnodes;

	/**
	 * Builds the Mermaid network results dumper.
	 *
	 * @param withSubnodes whether to include subnodes in the CFG panel
	 */
	public MermaidNetworkResults(
			boolean withSubnodes) {
		this.withSubnodes = withSubnodes;
	}

	@Override
	protected void dump(
			FileManager fileManager,
			SerializableGraph graph,
			String filename)
			throws IOException {
		fileManager.mkHtmlFile(filename + "-network", writer -> buildHtml(graph, writer));
		fileManager.usedHtmlViewer();
	}

	private void buildHtml(
			SerializableGraph graph,
			Writer writer)
			throws IOException {
		// DOT for the left CFG panel — stripped of abstract-state labels
		StringWriter graphWriter = new StringWriter();
		SerializableGraph bare = new SerializableGraph(
				graph.getName(), graph.getDescription(), graph.getNodes(), graph.getEdges(),
				Collections.emptySortedSet());
		DotGraph dot = bare.toDot();
		dot.dumpStripped(graphWriter);
		String graphText = graphWriter.toString();

		String graphTitle = "Graph: " + dot.getTitle();
		String graphDescription = "";
		if (StringUtils.isNotBlank(graph.getDescription()))
			graphDescription = "ID: " + graph.getDescription();

		// Determine top-level nodes (not subnodes of anything)
		Set<Integer> allSubNodes = new HashSet<>();
		for (SerializableNode n : graph.getNodes())
			allSubNodes.addAll(n.getSubNodes());

		Set<Integer> topLevelIds = graph.getNodes().stream()
				.map(SerializableNode::getId)
				.filter(id -> !allSubNodes.contains(id))
				.collect(Collectors.toSet());

		// Build map nodeId -> description
		Map<Integer, SerializableNodeDescription> descMap = new HashMap<>();
		for (SerializableNodeDescription d : graph.getDescriptions())
			descMap.put(d.getNodeId(), d);

		// Build per-node Mermaid divs
		StringBuilder descrs = new StringBuilder();
		topLevelIds.stream().sorted().forEach(nodeId -> {
			String nodeName = "node" + nodeId;
			SerializableNodeDescription desc = descMap.get(nodeId);
			String mermaid = (desc != null)
					? NetworkMermaidSerializer.toMermaid(desc)
					: NetworkMermaidSerializer.emptyDiagram();
			String endpointList = (desc != null)
					? NetworkMermaidSerializer.toEndpointListHtml(desc)
					: "";

			descrs.append("<div id=\"header-").append(nodeName)
					.append("\" class=\"header-info header-hidden\">\n");
			descrs.append("<div class=\"mermaid\">").append(mermaid).append("</div>\n");
			if (!endpointList.isEmpty())
				descrs.append(endpointList);
			descrs.append("</div>\n");
		});

		// Render via Thymeleaf
		TemplateEngine templateEngine = new TemplateEngine();
		StringTemplateResolver resolver = new StringTemplateResolver();
		resolver.setTemplateMode(TemplateMode.HTML);
		templateEngine.setTemplateResolver(resolver);

		String htmlTemplate = loadResourceTemplate("html-files-mermaid/viewer.html");

		Context context = new Context();
		context.setVariable("graphTitle", graphTitle);
		context.setVariable("graphDescription", graphDescription);
		context.setVariable("graphContent", graphText);
		context.setVariable("graphDescriptions", descrs.toString().trim());

		String html = templateEngine.process(htmlTemplate, context);
		writer.write(html);
	}

	private String loadResourceTemplate(
			String path)
			throws IOException {
		try (InputStream inputStream = getClass().getClassLoader().getResourceAsStream(path);
				Scanner scanner = new Scanner(inputStream, StandardCharsets.UTF_8)) {
			scanner.useDelimiter("\\A");
			return scanner.hasNext() ? scanner.next() : "";
		}
	}
}
