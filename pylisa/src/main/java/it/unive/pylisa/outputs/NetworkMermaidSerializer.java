package it.unive.pylisa.outputs;

import it.unive.lisa.outputs.serializableGraph.SerializableArray;
import it.unive.lisa.outputs.serializableGraph.SerializableNodeDescription;
import it.unive.lisa.outputs.serializableGraph.SerializableObject;
import it.unive.lisa.outputs.serializableGraph.SerializableString;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

/**
 * Utility class that converts a {@link SerializableNodeDescription} containing
 * a {@code "Network State"} field into a Mermaid {@code graph TD} diagram
 * string.
 */
public class NetworkMermaidSerializer {

	// palette[i] = { svcFill, stroke, svcTextColor, endpointFill }
	private static final String[][] PALETTE = {
			{ "#ff6b6b", "#c0392b", "#fff", "#ffb3b3" },
			{ "#48dbfb", "#0abde3", "#333", "#b8f4ff" },
			{ "#ff9ff3", "#f368e0", "#333", "#ffd6fc" },
			{ "#feca57", "#f9ca24", "#333", "#fff3b0" },
			{ "#54a0ff", "#2980b9", "#fff", "#b8d4ff" },
			{ "#5f27cd", "#341f97", "#fff", "#c8b0f5" },
			{ "#00d2d3", "#01abc2", "#333", "#a0f0f0" },
			{ "#ff9f43", "#e67e22", "#333", "#ffd5a0" }
	};

	/**
	 * Converts a {@link SerializableNodeDescription} into a Mermaid diagram
	 * string. The description is expected to contain a {@code "normal"} field,
	 * inside which a {@code "state"} field holds a {@code "Network State"}
	 * entry.
	 *
	 * @param nodeDesc the node description to convert
	 *
	 * @return the Mermaid diagram string
	 */
	public static String toMermaid(
			SerializableNodeDescription nodeDesc) {
		if (nodeDesc == null)
			return emptyDiagram();

		SerializableValue description = nodeDesc.getDescription();
		if (!(description instanceof SerializableObject))
			return emptyDiagram();

		SerializableObject topObj = (SerializableObject) description;

		SerializableValue normalVal = topObj.getFields().get("normal");
		if (!(normalVal instanceof SerializableObject))
			return emptyDiagram();

		SerializableObject normalObj = (SerializableObject) normalVal;

		SerializableValue stateVal = normalObj.getFields().get("state");
		if (!(stateVal instanceof SerializableObject))
			return emptyDiagram();

		SerializableObject stateObj = (SerializableObject) stateVal;

		SerializableValue netStateVal = stateObj.getFields().get("Network State");
		if (!(netStateVal instanceof SerializableObject))
			return emptyDiagram();

		SerializableObject netObj = (SerializableObject) netStateVal;
		if (netObj.getFields().isEmpty())
			return emptyDiagram();

		return renderNetworkObject(netObj);
	}

	/**
	 * Returns a placeholder Mermaid diagram for when no network state is
	 * available.
	 *
	 * @return the placeholder Mermaid string
	 */
	public static String emptyDiagram() {
		return "graph TD\n    empty[\"No network state\"]";
	}

	private static String renderNetworkObject(
			SerializableObject netObj) {
		StringBuilder sb = new StringBuilder("graph TD\n");

		// Assign a stable color-index to each allocation site
		Map<String, Integer> siteToIdx = new LinkedHashMap<>();
		int svcCounter = 0;
		for (String siteId : netObj.getFields().keySet())
			siteToIdx.put(siteId, svcCounter++);

		int ifaceCounter = 0;
		int prefCounter = 0;

		List<String> edges = new ArrayList<>();
		List<String> classAssignments = new ArrayList<>();

		for (Entry<String, SerializableValue> svcEntry : netObj.getFields().entrySet()) {
			String siteId = svcEntry.getKey();
			int idx = siteToIdx.get(siteId);
			String svcNodeId = "svc" + idx;

			if (!(svcEntry.getValue() instanceof SerializableObject))
				continue;
			SerializableObject svcObj = (SerializableObject) svcEntry.getValue();

			// Service node label = pretty basePath
			String basePath = "/";
			SerializableValue bpVal = svcObj.getFields().get("basePath");
			if (bpVal instanceof SerializableString)
				basePath = prettyBasePath(bpVal.toString());

			sb.append("\t").append(svcNodeId)
					.append("[\"").append(escapeLabel(basePath)).append("\"]\n");
			classAssignments.add("\tclass " + svcNodeId + " entity" + (idx % PALETTE.length) + "\n");

			// Interface (endpoint) nodes
			SerializableValue ifacesVal = svcObj.getFields().get("endpoints");
			List<String> endpointNodeIds = new ArrayList<>();
			if (ifacesVal instanceof SerializableObject) {
				SerializableObject ifacesObj = (SerializableObject) ifacesVal;
				for (Entry<String, SerializableValue> ifaceEntry : ifacesObj.getFields().entrySet()) {
					String ifaceNodeId = "iface" + ifaceCounter++;
					String label = buildEndpointLabel(ifaceEntry.getValue());
					sb.append("\t").append(ifaceNodeId)
							.append("[\"").append(escapeLabel(label)).append("\"]\n");
					edges.add("\t" + svcNodeId + " --> " + ifaceNodeId + "\n");
					endpointNodeIds.add(ifaceNodeId);
				}
			}
			if (!endpointNodeIds.isEmpty())
				classAssignments.add(
						"\tclass " + String.join(",", endpointNodeIds) + " endpoint" + (idx % PALETTE.length) + "\n");

			// Prefix / inner-service inclusion nodes
			SerializableValue innerVal = svcObj.getFields().get("innerNodes");
			if (innerVal instanceof SerializableObject) {
				SerializableObject innerObj = (SerializableObject) innerVal;
				for (Entry<String, SerializableValue> prefixEntry : innerObj.getFields().entrySet()) {
					String prefix = prettyValue(prefixEntry.getKey());
					String prefNodeId = "pref" + prefCounter++;
					sb.append("\t").append(prefNodeId)
							.append("([\"").append(escapeLabel(prefix)).append("\"])\n");
					edges.add("\t" + svcNodeId + " --> " + prefNodeId + "\n");
					classAssignments.add("\tclass " + prefNodeId + " prefstyle\n");

					// Value is an array of included site IDs
					if (prefixEntry.getValue() instanceof SerializableArray) {
						SerializableArray siteArr = (SerializableArray) prefixEntry.getValue();
						for (SerializableValue siteVal : siteArr.getElements()) {
							if (siteVal instanceof SerializableString) {
								String includedSite = siteVal.toString();
								Integer includedIdx = siteToIdx.get(includedSite);
								if (includedIdx != null)
									edges.add("\t" + prefNodeId + " --> svc" + includedIdx + "\n");
							}
						}
					}
				}
			}
		}

		// Edges
		for (String edge : edges)
			sb.append(edge);

		// classDef declarations — only emit as many as there are services
		int numColors = Math.min(siteToIdx.size(), PALETTE.length);
		for (int i = 0; i < numColors; i++) {
			String[] p = PALETTE[i];
			sb.append("\tclassDef entity").append(i)
					.append(" fill:").append(p[0])
					.append(",stroke:").append(p[1])
					.append(",color:").append(p[2]).append("\n");
			sb.append("\tclassDef endpoint").append(i)
					.append(" fill:").append(p[3])
					.append(",stroke:").append(p[1])
					.append(",color:#333\n");
		}
		sb.append("\tclassDef prefstyle fill:#ecf0f1,stroke:#bdc3c7,color:#2d3436\n");

		// class assignments
		for (String ca : classAssignments)
			sb.append(ca);

		return sb.toString();
	}

	private static String buildEndpointLabel(
			SerializableValue ifaceVal) {
		if (!(ifaceVal instanceof SerializableObject))
			return "?";
		SerializableObject obj = (SerializableObject) ifaceVal;
		// "resource" is the new key (was "interface" in the old model)
		SerializableValue resourceVal = obj.getFields().get("resource");
		if (!(resourceVal instanceof SerializableObject))
			return "?";
		SerializableObject resourceObj = (SerializableObject) resourceVal;

		String path = "";
		String methods = "";
		SerializableValue pathVal = resourceObj.getFields().get("path");
		SerializableValue methodsVal = resourceObj.getFields().get("methods");
		if (pathVal instanceof SerializableString)
			path = prettyValue(pathVal.toString());
		if (methodsVal != null)
			methods = prettyMethod(methodsVal.toString());

		if (methods.isEmpty())
			return path;
		return path + " [" + methods + "]";
	}

	static String prettyValue(
			String raw) {
		if ("_|_".equals(raw))
			return "\u22a5"; // ⊥
		if ("#TOP#".equals(raw))
			return "\u22a4"; // ⊤
		if ("empty".equals(raw))
			return "(empty)";
		if (raw != null && raw.length() >= 2 && raw.charAt(0) == '"' && raw.charAt(raw.length() - 1) == '"')
			return raw.substring(1, raw.length() - 1);
		return raw;
	}

	static String prettyBasePath(
			String raw) {
		String v = prettyValue(raw);
		return (v == null || v.isEmpty()) ? "/" : v;
	}

	static String prettyMethod(
			String raw) {
		String v = prettyValue(raw);
		if (v == null || "null".equals(v) || "?".equals(v))
			return "";
		return v;
	}

	static String escapeLabel(
			String label) {
		if (label == null)
			return "";
		return label.replace("\"", "'");
	}
}
