package it.unive.pylisa.outputs;

import it.unive.lisa.outputs.serializableGraph.SerializableArray;
import it.unive.lisa.outputs.serializableGraph.SerializableNodeDescription;
import it.unive.lisa.outputs.serializableGraph.SerializableObject;
import it.unive.lisa.outputs.serializableGraph.SerializableString;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

/**
 * Utility class that converts a {@link SerializableNodeDescription} containing
 * a {@code "Network State"} field into a Mermaid {@code graph TD} diagram
 * string. Supports HTTP (FastAPI/Flask), S3, and ROS2 protocol sections, as
 * well as cross-protocol boundary edges captured by the second-pass handler
 * analysis.
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

	// S3 color
	private static final String S3_FILL = "#f6ae2d";
	private static final String S3_STROKE = "#c47d00";
	private static final String S3_ENDPOINT_FILL = "#fde8a0";

	// ROS2 color
	private static final String ROS2_FILL = "#6ab187";
	private static final String ROS2_STROKE = "#3d7a57";
	private static final String ROS2_ENDPOINT_FILL = "#c0e8d0";

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

		// Extract handlerMap and crossEdges (may be absent in older states)
		SerializableValue handlerMapVal = stateObj.getFields().get("Handler Map");
		SerializableValue crossEdgesVal = stateObj.getFields().get("Cross Edges");

		return renderNetworkObject(netObj, handlerMapVal, crossEdgesVal);
	}

	/**
	 * Converts a {@link SerializableValue} representing a
	 * {@code NetworkAbstractState} directly (i.e. the "state" object already
	 * unwrapped from the "normal"/"state" wrapper) into a Mermaid diagram
	 * string. Use this variant when you have the lattice representation
	 * directly, without the surrounding {@code AnalysisState} wrapper.
	 *
	 * @param stateVal the serialised {@code NetworkAbstractState}
	 *
	 * @return the Mermaid diagram string
	 */
	public static String toMermaidFromState(
			SerializableValue stateVal) {
		if (!(stateVal instanceof SerializableObject))
			return emptyDiagram();
		SerializableObject stateObj = (SerializableObject) stateVal;
		SerializableValue netStateVal = stateObj.getFields().get("Network State");
		if (!(netStateVal instanceof SerializableObject))
			return emptyDiagram();
		SerializableObject netObj = (SerializableObject) netStateVal;
		if (netObj.getFields().isEmpty())
			return emptyDiagram();
		SerializableValue handlerMapVal = stateObj.getFields().get("Handler Map");
		SerializableValue crossEdgesVal = stateObj.getFields().get("Cross Edges");
		return renderNetworkObject(netObj, handlerMapVal, crossEdgesVal);
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
			SerializableObject netObj,
			SerializableValue handlerMapVal,
			SerializableValue crossEdgesVal) {
		StringBuilder sb = new StringBuilder("graph TD\n");
		List<String> edges = new ArrayList<>();
		List<String> classAssignments = new ArrayList<>();

		// Assign stable color indices per allocation site across all protocols
		Map<String, Integer> siteToIdx = new LinkedHashMap<>();
		int svcCounter = 0;

		// Collect all HTTP site IDs
		SerializableValue httpVal = netObj.getFields().get("http");
		if (httpVal instanceof SerializableObject) {
			SerializableObject httpObj = (SerializableObject) httpVal;
			for (String siteId : httpObj.getFields().keySet())
				siteToIdx.put(siteId, svcCounter++);
		}

		// Track node IDs for cross-protocol edge rendering
		// path label → iface node ID
		Map<String, String> httpEndpointNodeIds = new LinkedHashMap<>();
		// resource label → s3/ros2 resource node ID
		Map<String, String> resourceNodeIds = new LinkedHashMap<>();

		int[] ifaceCounter = { 0 };
		int[] prefCounter = { 0 };

		// Render HTTP section
		if (httpVal instanceof SerializableObject) {
			SerializableObject httpObj = (SerializableObject) httpVal;
			renderHttpNodes(httpObj, siteToIdx, sb, edges, classAssignments,
					ifaceCounter, prefCounter, httpEndpointNodeIds);
		}

		// Render S3 section
		SerializableValue s3Val = netObj.getFields().get("s3");
		int s3Counter = 0;
		if (s3Val instanceof SerializableObject) {
			SerializableObject s3Obj = (SerializableObject) s3Val;
			for (Entry<String, SerializableValue> clientEntry : s3Obj.getFields().entrySet()) {
				if (!(clientEntry.getValue() instanceof SerializableObject))
					continue;
				SerializableObject clientObj = (SerializableObject) clientEntry.getValue();

				String clientNodeId = "s3client" + s3Counter;
				String clientLabel = "S3 Client";
				SerializableValue clientIdVal = clientObj.getFields().get("clientId");
				if (clientIdVal instanceof SerializableString) {
					String cid = prettyValue(clientIdVal.toString());
					if (cid != null && !cid.isEmpty() && !"\u22a5".equals(cid))
						clientLabel = "S3 Client: " + cid;
				}

				sb.append("\t").append(clientNodeId)
						.append("[\"").append(escapeLabel(clientLabel)).append("\"]\n");
				classAssignments.add("\tclass " + clientNodeId + " s3entity\n");

				// S3 endpoints
				SerializableValue endpointsVal = clientObj.getFields().get("endpoints");
				List<String> s3EndpointIds = new ArrayList<>();
				if (endpointsVal instanceof SerializableObject) {
					SerializableObject endpointsObj = (SerializableObject) endpointsVal;
					for (Entry<String, SerializableValue> epEntry : endpointsObj.getFields().entrySet()) {
						String epNodeId = "s3ep" + s3Counter + "_" + s3EndpointIds.size();
						String label = buildS3EndpointLabel(epEntry.getValue());
						sb.append("\t").append(epNodeId)
								.append("[\"").append(escapeLabel(label)).append("\"]\n");
						edges.add("\t" + clientNodeId + " --> " + epNodeId + "\n");
						s3EndpointIds.add(epNodeId);
						// Track for cross-protocol edges: use the raw key as
						// lookup
						resourceNodeIds.put(epEntry.getKey(), epNodeId);
						resourceNodeIds.put(label, epNodeId);
					}
				}
				if (!s3EndpointIds.isEmpty())
					classAssignments.add(
							"\tclass " + String.join(",", s3EndpointIds) + " s3endpoint\n");

				s3Counter++;
			}
		}

		// Render ROS2 section
		SerializableValue ros2Val = netObj.getFields().get("ros2");
		int ros2Counter = 0;
		if (ros2Val instanceof SerializableObject) {
			SerializableObject ros2Obj = (SerializableObject) ros2Val;
			for (Entry<String, SerializableValue> nodeEntry : ros2Obj.getFields().entrySet()) {
				if (!(nodeEntry.getValue() instanceof SerializableObject))
					continue;
				SerializableObject nodeObj = (SerializableObject) nodeEntry.getValue();

				String ros2NodeId = "ros2node" + ros2Counter;
				String ros2Label = "ROS2 Node";
				SerializableValue nodeNameVal = nodeObj.getFields().get("nodeName");
				if (nodeNameVal instanceof SerializableString) {
					String nn = prettyValue(nodeNameVal.toString());
					if (nn != null && !nn.isEmpty() && !"\u22a5".equals(nn))
						ros2Label = nn;
				}

				sb.append("\t").append(ros2NodeId)
						.append("[\"").append(escapeLabel(ros2Label)).append("\"]\n");
				classAssignments.add("\tclass " + ros2NodeId + " ros2entity\n");

				// ROS2 topic endpoints
				SerializableValue endpointsVal = nodeObj.getFields().get("endpoints");
				List<String> ros2EndpointIds = new ArrayList<>();
				if (endpointsVal instanceof SerializableObject) {
					SerializableObject endpointsObj = (SerializableObject) endpointsVal;
					for (Entry<String, SerializableValue> epEntry : endpointsObj.getFields().entrySet()) {
						String epNodeId = "ros2ep" + ros2Counter + "_" + ros2EndpointIds.size();
						String label = buildROS2EndpointLabel(epEntry.getValue());
						sb.append("\t").append(epNodeId)
								.append("[\"").append(escapeLabel(label)).append("\"]\n");
						edges.add("\t" + ros2NodeId + " --> " + epNodeId + "\n");
						ros2EndpointIds.add(epNodeId);
						resourceNodeIds.put(epEntry.getKey(), epNodeId);
						resourceNodeIds.put(label, epNodeId);
					}
				}
				if (!ros2EndpointIds.isEmpty())
					classAssignments.add(
							"\tclass " + String.join(",", ros2EndpointIds) + " ros2endpoint\n");

				ros2Counter++;
			}
		}

		// Render cross-protocol dashed edges
		renderCrossProtocolEdges(handlerMapVal, crossEdgesVal, httpEndpointNodeIds, resourceNodeIds,
				sb, edges);

		// Edges
		for (String edge : edges)
			sb.append(edge);

		// classDef declarations for HTTP
		int numHttpColors = Math.min(siteToIdx.size(), PALETTE.length);
		for (int i = 0; i < numHttpColors; i++) {
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

		// classDef for S3
		if (s3Counter > 0) {
			sb.append("\tclassDef s3entity fill:").append(S3_FILL)
					.append(",stroke:").append(S3_STROKE).append(",color:#333\n");
			sb.append("\tclassDef s3endpoint fill:").append(S3_ENDPOINT_FILL)
					.append(",stroke:").append(S3_STROKE).append(",color:#333\n");
		}

		// classDef for ROS2
		if (ros2Counter > 0) {
			sb.append("\tclassDef ros2entity fill:").append(ROS2_FILL)
					.append(",stroke:").append(ROS2_STROKE).append(",color:#fff\n");
			sb.append("\tclassDef ros2endpoint fill:").append(ROS2_ENDPOINT_FILL)
					.append(",stroke:").append(ROS2_STROKE).append(",color:#333\n");
		}

		// class assignments
		for (String ca : classAssignments)
			sb.append(ca);

		return sb.toString();
	}

	/**
	 * Renders dashed cross-protocol edges from HTTP endpoint nodes to S3/ROS2
	 * resource nodes, using the {@code handlerMap} and {@code crossEdges}
	 * lattice fields serialised into the network state.
	 */
	private static void renderCrossProtocolEdges(
			SerializableValue handlerMapVal,
			SerializableValue crossEdgesVal,
			Map<String, String> httpEndpointNodeIds,
			Map<String, String> resourceNodeIds,
			StringBuilder sb,
			List<String> edges) {
		if (!(handlerMapVal instanceof SerializableObject))
			return;
		if (!(crossEdgesVal instanceof SerializableObject))
			return;
		SerializableObject handlerMapObj = (SerializableObject) handlerMapVal;
		SerializableObject crossEdgesObj = (SerializableObject) crossEdgesVal;

		for (Entry<String, SerializableValue> hmEntry : handlerMapObj.getFields().entrySet()) {
			// hmEntry.getKey() = EndpointKey.toString():
			// HTTP: "protocol:role:method:path@siteId"
			// Other: "protocol:role:path@siteId"
			// The siteId itself may contain '@' (e.g.
			// "heap[s]:pp@'file':line:col"),
			// so use indexOf (first '@') to find the separator before the
			// siteId.
			String endpointKeyStr = hmEntry.getKey();

			int atIdx = endpointKeyStr.indexOf('@');
			String withoutSite = atIdx > 0 ? endpointKeyStr.substring(0, atIdx) : endpointKeyStr;
			int firstColon = withoutSite.indexOf(':');
			int secondColon = firstColon >= 0 ? withoutSite.indexOf(':', firstColon + 1) : -1;
			if (secondColon < 0)
				continue;
			String protocol = withoutSite.substring(0, firstColon);

			// For HTTP keys there is a third colon separating method from path
			String methodPart = null;
			String rawPath;
			if ("http".equals(protocol)) {
				int thirdColon = withoutSite.indexOf(':', secondColon + 1);
				if (thirdColon >= 0) {
					methodPart = withoutSite.substring(secondColon + 1, thirdColon);
					rawPath = withoutSite.substring(thirdColon + 1);
				} else {
					// Fallback: no method segment (old format)
					rawPath = withoutSite.substring(secondColon + 1);
				}
			} else {
				rawPath = withoutSite.substring(secondColon + 1);
			}

			String prettyPath = prettyValue(rawPath);

			// Find the HTTP endpoint Mermaid node — try method:path first, then
			// path alone
			String httpNodeId = null;
			if (methodPart != null)
				httpNodeId = httpEndpointNodeIds.get(methodPart + ":" + prettyPath);
			if (httpNodeId == null)
				httpNodeId = httpEndpointNodeIds.get(prettyPath);
			if (httpNodeId == null)
				httpNodeId = httpEndpointNodeIds.get(rawPath);
			if (httpNodeId == null)
				continue;

			// Collect handler names for this endpoint
			SerializableValue handlersVal = hmEntry.getValue();
			List<String> handlerNames = extractStringElements(handlersVal);

			for (String handlerName : handlerNames) {
				// Look up crossEdges for this handler
				SerializableValue refsVal = crossEdgesObj.getFields().get(handlerName);
				List<String> resourceRefs = extractStringElements(refsVal);

				for (String refStr : resourceRefs) {
					// refStr = ProtocolResourceRef.toString() e.g.
					// "s3:bucket/key[CONSUMER]"
					// Extract role from the last '[...]'
					int bracketOpen = refStr.lastIndexOf('[');
					int bracketClose = refStr.lastIndexOf(']');
					String roleStr = (bracketOpen >= 0 && bracketClose > bracketOpen)
							? refStr.substring(bracketOpen + 1, bracketClose)
							: "";

					// Extract resource identifier (between "protocol:" and
					// "[role]").
					// ConstantPropagation values are stored as "\"value\"" with
					// surrounding
					// quotes, so normalise by stripping all '"' before
					// comparing against
					// the pretty-printed labels stored in resourceNodeIds.
					String resourcePart = refStr.substring(
							refStr.indexOf(':') + 1,
							bracketOpen > 0 ? bracketOpen : refStr.length());
					String normalizedRef = resourcePart.replace("\"", "");

					// Direct exact lookup, then normalised lookup
					String resNodeId = resourceNodeIds.get(refStr);
					if (resNodeId == null)
						resNodeId = resourceNodeIds.get(normalizedRef);
					if (resNodeId == null) {
						// Fuzzy: strip role suffix (" [...]") from each key,
						// then compare
						for (Entry<String, String> rEntry : resourceNodeIds.entrySet()) {
							String normalizedKey = rEntry.getKey()
									.replaceFirst(" \\[.*\\]$", "")
									.replace("\"", "");
							if (normalizedKey.equals(normalizedRef)) {
								resNodeId = rEntry.getValue();
								break;
							}
						}
					}
					if (resNodeId == null)
						continue;

					// Add dashed arrow from HTTP endpoint to resource node
					String arrowLabel = roleStr.isEmpty() ? "" : "|" + roleStr + "|";
					edges.add("\t" + httpNodeId + " -.->" + arrowLabel + " " + resNodeId + "\n");
				}
			}
		}
	}

	/** Extracts string elements from a serialized set/array value. */
	private static List<String> extractStringElements(
			SerializableValue val) {
		List<String> result = new ArrayList<>();
		if (val instanceof SerializableArray) {
			for (SerializableValue elem : ((SerializableArray) val).getElements())
				result.add(elem.toString());
		} else if (val instanceof SerializableObject) {
			// Some set representations serialize as objects
			for (Entry<String, SerializableValue> e : ((SerializableObject) val).getFields().entrySet())
				result.add(e.getKey());
		}
		return result;
	}

	private static void renderHttpNodes(
			SerializableObject netObj,
			Map<String, Integer> siteToIdx,
			StringBuilder sb,
			List<String> edges,
			List<String> classAssignments,
			int[] ifaceCounter,
			int[] prefCounter,
			Map<String, String> httpEndpointNodeIds) {
		for (Entry<String, SerializableValue> svcEntry : netObj.getFields().entrySet()) {
			String siteId = svcEntry.getKey();
			Integer idxObj = siteToIdx.get(siteId);
			if (idxObj == null)
				continue;
			int idx = idxObj;
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
					String ifaceNodeId = "iface" + ifaceCounter[0]++;
					String label = buildEndpointLabel(ifaceEntry.getValue());
					sb.append("\t").append(ifaceNodeId)
							.append("[\"").append(escapeLabel(label)).append("\"]\n");
					edges.add("\t" + svcNodeId + " --> " + ifaceNodeId + "\n");
					endpointNodeIds.add(ifaceNodeId);

					// Track: raw key, pretty key, and method:prettyPath form
					// for cross-edge lookup.
					// The serialised key is "METHOD:pathString"
					// (EndpointMapKey.toString()).
					String rawKey = ifaceEntry.getKey();
					httpEndpointNodeIds.put(rawKey, ifaceNodeId);
					httpEndpointNodeIds.put(prettyValue(rawKey), ifaceNodeId);
					// Extract method and path from the key
					// ("METHOD:pathString")
					int keyColon = rawKey.indexOf(':');
					if (keyColon > 0) {
						String keyMethod = rawKey.substring(0, keyColon);
						String keyPathRaw = rawKey.substring(keyColon + 1);
						String keyPathPretty = prettyValue(keyPathRaw);
						httpEndpointNodeIds.put(keyMethod + ":" + keyPathPretty, ifaceNodeId);
						httpEndpointNodeIds.put(keyPathPretty, ifaceNodeId);
					}
					// Also track by path-only part of the label (strip " [GET]"
					// etc.)
					String pathOnly = label.contains(" [") ? label.substring(0, label.lastIndexOf(" [")) : label;
					httpEndpointNodeIds.put(pathOnly, ifaceNodeId);
				}
			}
			if (!endpointNodeIds.isEmpty())
				classAssignments.add(
						"\tclass " + String.join(",", endpointNodeIds) + " endpoint" + (idx % PALETTE.length)
								+ "\n");

			// Prefix / inner-service inclusion nodes
			SerializableValue innerVal = svcObj.getFields().get("innerNodes");
			if (innerVal instanceof SerializableObject) {
				SerializableObject innerObj = (SerializableObject) innerVal;
				for (Entry<String, SerializableValue> prefixEntry : innerObj.getFields().entrySet()) {
					String prefix = prettyBasePath(prefixEntry.getKey());
					String prefNodeId = "pref" + prefCounter[0]++;
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
	}

	private static String buildEndpointLabel(
			SerializableValue ifaceVal) {
		if (!(ifaceVal instanceof SerializableObject))
			return "?";
		SerializableObject obj = (SerializableObject) ifaceVal;
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

	private static String buildS3EndpointLabel(
			SerializableValue epVal) {
		if (!(epVal instanceof SerializableObject))
			return "?";
		SerializableObject obj = (SerializableObject) epVal;
		SerializableValue resourceVal = obj.getFields().get("resource");
		SerializableValue roleVal = obj.getFields().get("role");

		String bucket = "";
		String key = "";
		String role = "";
		if (resourceVal instanceof SerializableObject) {
			SerializableObject resourceObj = (SerializableObject) resourceVal;
			SerializableValue bv = resourceObj.getFields().get("bucket");
			SerializableValue kv = resourceObj.getFields().get("key");
			if (bv instanceof SerializableString)
				bucket = prettyValue(bv.toString());
			if (kv instanceof SerializableString)
				key = prettyValue(kv.toString());
		}
		if (roleVal != null)
			role = prettyValue(roleVal.toString());

		StringBuilder label = new StringBuilder();
		if (!bucket.isEmpty())
			label.append(bucket);
		if (!key.isEmpty() && !"\u22a5".equals(key))
			label.append("/").append(key);
		if (!role.isEmpty())
			label.append(" [").append(role).append("]");
		return label.length() > 0 ? label.toString() : "?";
	}

	private static String buildROS2EndpointLabel(
			SerializableValue epVal) {
		if (!(epVal instanceof SerializableObject))
			return "?";
		SerializableObject obj = (SerializableObject) epVal;
		SerializableValue resourceVal = obj.getFields().get("resource");
		SerializableValue roleVal = obj.getFields().get("role");

		String topic = "";
		String role = "";
		if (resourceVal instanceof SerializableObject) {
			SerializableObject resourceObj = (SerializableObject) resourceVal;
			SerializableValue tv = resourceObj.getFields().get("topic");
			if (tv instanceof SerializableString)
				topic = prettyValue(tv.toString());
		}
		if (roleVal != null)
			role = prettyValue(roleVal.toString());

		if (role.isEmpty())
			return topic.isEmpty() ? "?" : topic;
		return topic + " [" + role + "]";
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

	// -----------------------------------------------------------------------
	// Endpoint list
	// -----------------------------------------------------------------------

	/**
	 * Extracts the list of HTTP endpoints from a
	 * {@link SerializableNodeDescription} and returns an HTML table fragment
	 * (inline-styled, no external CSS needed). Returns an empty string when no
	 * endpoints are found.
	 *
	 * @param nodeDesc the node description
	 *
	 * @return the HTML table fragment, or {@code ""} if none
	 */
	public static String toEndpointListHtml(
			SerializableNodeDescription nodeDesc) {
		if (nodeDesc == null)
			return "";
		SerializableValue description = nodeDesc.getDescription();
		if (!(description instanceof SerializableObject))
			return "";
		SerializableObject topObj = (SerializableObject) description;
		SerializableValue normalVal = topObj.getFields().get("normal");
		if (!(normalVal instanceof SerializableObject))
			return "";
		SerializableObject normalObj = (SerializableObject) normalVal;
		return toEndpointListHtmlFromState(normalObj.getFields().get("state"));
	}

	/**
	 * Extracts the list of HTTP endpoints from a serialised
	 * {@code NetworkAbstractState} and returns an HTML table fragment.
	 *
	 * @param stateVal the serialised state value
	 *
	 * @return the HTML table fragment, or {@code ""} if none
	 */
	public static String toEndpointListHtmlFromState(
			SerializableValue stateVal) {
		if (!(stateVal instanceof SerializableObject))
			return "";
		SerializableObject stateObj = (SerializableObject) stateVal;
		SerializableValue netStateVal = stateObj.getFields().get("Network State");
		if (!(netStateVal instanceof SerializableObject))
			return "";
		SerializableObject netObj = (SerializableObject) netStateVal;
		if (netObj.getFields().isEmpty())
			return "";
		return buildEndpointListHtml(netObj, stateObj.getFields().get("Handler Map"));
	}

	private static String buildEndpointListHtml(
			SerializableObject netObj,
			SerializableValue handlerMapVal) {
		// Build lookup: prettyEndpointKey (path segment from handler map) →
		// handlers
		Map<String, List<String>> keyToHandlers = new LinkedHashMap<>();
		if (handlerMapVal instanceof SerializableObject) {
			SerializableObject handlerMapObj = (SerializableObject) handlerMapVal;
			for (Entry<String, SerializableValue> hmEntry : handlerMapObj.getFields().entrySet()) {
				// ekStr format:
				// HTTP: "protocol:role:method:path@siteId"
				// Other: "protocol:role:path@siteId"
				String ekStr = hmEntry.getKey();
				int atIdx = ekStr.indexOf('@');
				String withoutSite = atIdx > 0 ? ekStr.substring(0, atIdx) : ekStr;
				int fc = withoutSite.indexOf(':');
				if (fc < 0)
					continue;
				if (!"http".equals(withoutSite.substring(0, fc)))
					continue;
				int sc = withoutSite.indexOf(':', fc + 1);
				if (sc < 0)
					continue;
				// For HTTP, there is a third colon: "http:role:method:path"
				int tc = withoutSite.indexOf(':', sc + 1);
				String methodPart;
				String keyPath;
				if (tc >= 0) {
					methodPart = withoutSite.substring(sc + 1, tc);
					keyPath = prettyValue(withoutSite.substring(tc + 1));
				} else {
					// Fallback: old format without method
					methodPart = "";
					keyPath = prettyValue(withoutSite.substring(sc + 1));
				}
				String mapKey = methodPart.isEmpty() ? keyPath : methodPart + ":" + keyPath;
				keyToHandlers.computeIfAbsent(mapKey, k -> new ArrayList<>())
						.addAll(extractStringElements(hmEntry.getValue()));
			}
		}

		// Collect endpoint rows via recursive traversal of the innerNodes
		// hierarchy
		SerializableValue httpVal = netObj.getFields().get("http");
		if (!(httpVal instanceof SerializableObject))
			return "";
		SerializableObject httpObj = (SerializableObject) httpVal;

		// Build sites map: siteId → svcObj
		Map<String, SerializableObject> sites = new LinkedHashMap<>();
		for (Entry<String, SerializableValue> e : httpObj.getFields().entrySet())
			if (e.getValue() instanceof SerializableObject)
				sites.put(e.getKey(), (SerializableObject) e.getValue());

		// Find child site IDs (referenced by any innerNodes) so we only start
		// from roots
		Set<String> childSiteIds = new LinkedHashSet<>();
		for (SerializableObject svcObj : sites.values()) {
			SerializableValue innerVal = svcObj.getFields().get("innerNodes");
			if (!(innerVal instanceof SerializableObject))
				continue;
			for (Entry<String, SerializableValue> ie : ((SerializableObject) innerVal).getFields().entrySet()) {
				if (ie.getValue() instanceof SerializableArray)
					for (SerializableValue sv : ((SerializableArray) ie.getValue()).getElements())
						childSiteIds.add(sv.toString());
			}
		}

		List<String[]> rows = new ArrayList<>();
		for (String siteId : sites.keySet())
			if (!childSiteIds.contains(siteId))
				collectEndpointRows(siteId, "", sites, keyToHandlers, rows, new LinkedHashSet<>());

		if (rows.isEmpty())
			return "";

		rows.sort((
				a,
				b) -> {
			int c = a[0].compareTo(b[0]);
			return c != 0 ? c : a[1].compareTo(b[1]);
		});

		StringBuilder sb = new StringBuilder();
		sb.append(buildEndpointSummaryHtml(rows));
		sb.append("<div style=\"margin-top:16px;font-family:sans-serif;\">\n");
		sb.append("  <h3 style=\"color:#2d3436;font-size:1em;margin-bottom:8px;\">Endpoints</h3>\n");
		sb.append("  <table style=\"border-collapse:collapse;width:100%;font-size:.9em;\">\n");
		sb.append("    <thead><tr>")
				.append(th("Method")).append(th("Path")).append(th("Handler"))
				.append("</tr></thead>\n    <tbody>\n");
		for (String[] row : rows) {
			sb.append("      <tr>")
					.append("<td style=\"padding:4px 8px;border:1px solid #dfe6e9;background:")
					.append(methodColor(row[0]))
					.append(";font-weight:bold;font-family:monospace;\">").append(escapeHtml(row[0])).append("</td>")
					.append("<td style=\"padding:4px 8px;border:1px solid #dfe6e9;font-family:monospace;\">")
					.append(escapeHtml(row[1])).append("</td>")
					.append("<td style=\"padding:4px 8px;border:1px solid #dfe6e9;font-family:monospace;\">")
					.append(escapeHtml(row[2])).append("</td>")
					.append("</tr>\n");
		}
		sb.append("    </tbody>\n  </table>\n</div>\n");
		return sb.toString();
	}

	private static String th(
			String label) {
		return "<th style=\"text-align:left;padding:4px 8px;background:#ecf0f1;border:1px solid #dfe6e9;\">"
				+ label + "</th>";
	}

	/**
	 * Builds the per-HTTP-method summary block rendered above the endpoint
	 * table. Counts are taken from the collected endpoint rows (first column)
	 * so the summary stays in sync with whatever the endpoint table shows.
	 * Methods are listed in a canonical order; any method seen in rows but
	 * not in that order is appended alphabetically afterwards under "Other".
	 */
	private static String buildEndpointSummaryHtml(
			List<String[]> rows) {
		java.util.Map<String, Integer> counts = new java.util.LinkedHashMap<>();
		int total = 0;
		for (String[] r : rows) {
			String m = r[0] == null ? "" : r[0].toUpperCase();
			counts.merge(m, 1, Integer::sum);
			total++;
		}
		String[] canonical = { "GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS", "HEAD" };
		StringBuilder sb = new StringBuilder();
		sb.append("<div style=\"margin-top:16px;font-family:sans-serif;\">\n");
		sb.append("  <h3 style=\"color:#2d3436;font-size:1em;margin-bottom:8px;\">Summary</h3>\n");
		sb.append("  <div style=\"display:flex;flex-wrap:wrap;gap:8px;margin-bottom:8px;\">\n");
		for (String m : canonical) {
			int c = counts.getOrDefault(m, 0);
			sb.append(methodBadge(m, c));
			counts.remove(m);
		}
		// any non-canonical methods seen (unusual) — append as-is
		for (java.util.Map.Entry<String, Integer> e : counts.entrySet())
			sb.append(methodBadge(e.getKey().isEmpty() ? "OTHER" : e.getKey(), e.getValue()));
		sb.append("    <div style=\"padding:6px 12px;border-radius:4px;background:#2d3436;color:#fff;")
				.append("font-family:monospace;font-weight:bold;\">TOTAL: ").append(total).append("</div>\n");
		sb.append("  </div>\n</div>\n");
		return sb.toString();
	}

	private static String methodBadge(
			String method,
			int count) {
		return "    <div style=\"padding:6px 12px;border-radius:4px;background:"
				+ methodColor(method)
				+ ";font-family:monospace;font-weight:bold;color:#2d3436;\">"
				+ escapeHtml(method) + ": " + count + "</div>\n";
	}

	/**
	 * Recursively collects endpoint rows for a site and all its inner
	 * (included) child sites, accumulating the full prefix from the root down.
	 *
	 * @param siteId        the current site to process
	 * @param prefixSoFar   accumulated path prefix from the parent (already
	 *                          includes basePath of all ancestors + include
	 *                          prefixes)
	 * @param sites         all HTTP sites keyed by allocation-site ID
	 * @param keyToHandlers handler lookup
	 * @param rows          output accumulator
	 * @param visited       cycle guard
	 */
	private static void collectEndpointRows(
			String siteId,
			String prefixSoFar,
			Map<String, SerializableObject> sites,
			Map<String, List<String>> keyToHandlers,
			List<String[]> rows,
			Set<String> visited) {
		if (!visited.add(siteId))
			return;
		SerializableObject svcObj = sites.get(siteId);
		if (svcObj == null)
			return;

		// Incorporate this site's own basePath into the running prefix.
		// Skip "/" to avoid producing "//foo" for root apps.
		String basePath = "";
		SerializableValue bpVal = svcObj.getFields().get("basePath");
		if (bpVal instanceof SerializableString) {
			String bp = prettyBasePath(bpVal.toString());
			if (!"/".equals(bp))
				basePath = bp;
		}
		String currentPrefix = normalizePath(prefixSoFar + "/" + basePath);

		// Collect direct endpoints at this level
		SerializableValue ifacesVal = svcObj.getFields().get("endpoints");
		if (ifacesVal instanceof SerializableObject) {
			for (Entry<String, SerializableValue> ifaceEntry : ((SerializableObject) ifacesVal).getFields()
					.entrySet()) {
				if (!(ifaceEntry.getValue() instanceof SerializableObject))
					continue;
				SerializableObject ifaceObj = (SerializableObject) ifaceEntry.getValue();
				SerializableValue resourceVal = ifaceObj.getFields().get("resource");
				if (!(resourceVal instanceof SerializableObject))
					continue;
				SerializableObject resourceObj = (SerializableObject) resourceVal;

				String epPath = "";
				SerializableValue pathVal = resourceObj.getFields().get("path");
				if (pathVal instanceof SerializableString)
					epPath = prettyValue(pathVal.toString());

				String fullPath = normalizePath(currentPrefix + "/" + epPath);

				// The serialised map key is "METHOD:pathString"
				// (EndpointMapKey.toString()).
				// Extract the method directly from the key so each row shows
				// exactly one method.
				String rawKey = ifaceEntry.getKey();
				int keyColon = rawKey.indexOf(':');
				String methodFromKey;
				String keyPathRaw;
				if (keyColon > 0) {
					methodFromKey = rawKey.substring(0, keyColon);
					keyPathRaw = rawKey.substring(keyColon + 1);
				} else {
					// Fallback: no method in key (old format)
					methodFromKey = prettyMethod(
							resourceObj.getFields().get("methods") != null
									? resourceObj.getFields().get("methods").toString()
									: "");
					keyPathRaw = rawKey;
				}

				// Handler lookup: try "METHOD:prettyPath", then "prettyPath",
				// then epPath
				String keyPathPretty = prettyValue(keyPathRaw);
				String mapKey = methodFromKey.isEmpty() ? keyPathPretty
						: methodFromKey + ":" + keyPathPretty;
				List<String> handlers = keyToHandlers.getOrDefault(mapKey,
						keyToHandlers.getOrDefault(keyPathPretty,
								keyToHandlers.getOrDefault(epPath, List.of())));
				rows.add(new String[] { methodFromKey, fullPath, String.join(", ", handlers) });
			}
		}

		// Recurse into inner (included) child sites, prepending the include
		// prefix
		SerializableValue innerVal = svcObj.getFields().get("innerNodes");
		if (innerVal instanceof SerializableObject) {
			for (Entry<String, SerializableValue> innerEntry : ((SerializableObject) innerVal).getFields()
					.entrySet()) {
				String includePrefix = prettyValue(innerEntry.getKey());
				if (includePrefix == null || "/".equals(includePrefix))
					includePrefix = "";
				String childPrefixSoFar = normalizePath(currentPrefix + "/" + includePrefix);
				if (innerEntry.getValue() instanceof SerializableArray) {
					for (SerializableValue sv : ((SerializableArray) innerEntry.getValue()).getElements()) {
						Set<String> childVisited = new LinkedHashSet<>(visited);
						collectEndpointRows(sv.toString(), childPrefixSoFar, sites, keyToHandlers, rows,
								childVisited);
					}
				}
			}
		}
	}

	private static String normalizePath(
			String path) {
		if (path == null)
			return "/";
		String n = path.replaceAll("/+", "/");
		return n.startsWith("/") ? n : "/" + n;
	}

	private static String methodColor(
			String method) {
		if (method == null)
			return "#b2bec3";
		switch (method.toUpperCase()) {
		case "GET":
			return "#55efc4";
		case "POST":
			return "#74b9ff";
		case "PUT":
			return "#fdcb6e";
		case "DELETE":
			return "#ff7675";
		case "PATCH":
			return "#a29bfe";
		default:
			return "#b2bec3";
		}
	}

	private static String escapeHtml(
			String s) {
		if (s == null)
			return "";
		return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
	}
}
