package it.unive.ros.application;

import com.ibm.icu.impl.Pair;
import it.unive.lisa.util.file.FileManager;
import it.unive.ros.models.rclpy.*;
import it.unive.ros.permissions.jaxb.Grant;
import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class ROSApplication {

	private String workDir;
	private final RosComputationalGraph ROSComputationalGraph;

	private Map<String, Grant> permissionsGrants = new HashMap<>();

	public void setWorkDir(
			String workDir) {
		this.workDir = workDir;
	}

	public String getWorkDir() {
		return workDir;
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph) {
		this.ROSComputationalGraph = ROSComputationalGraph;
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph,
			String workDir) {
		this.ROSComputationalGraph = ROSComputationalGraph;
		this.workDir = workDir;
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph,
			Map<String, Grant> permissionsGrants,
			String workDir) {
		this.ROSComputationalGraph = ROSComputationalGraph;
		this.workDir = workDir;
		this.permissionsGrants = permissionsGrants;
	}

	public RosComputationalGraph getROSComputationalGraph() {
		return ROSComputationalGraph;
	}

	public void dumpPermissions() throws Exception {
		for (Node n : this.getROSComputationalGraph().getNodes()) {
			n.dumpPermissions(workDir);
		}
	}

	public void dumpGraph() throws Exception {
		StringBuilder dotGraph = new StringBuilder(
				"digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
		// DUMP Services
		int i = 0;
		for (Node n : ROSComputationalGraph.getNodes()) {
			for (Service s : n.getServices()) {
				dotGraph.append("subgraph cluster_" + i
						+ " { style=filled;fillcolor=white;color=blue;penwidth=2;fontcolor=blue;label=\"" + s.getName()
						+ "\";").append("\"" + s.getName() + "Reply\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];").append("\"" + s.getName() + "Request\"")
						.append("[shape=box,style=filled];}");
				i++;
			}
			dotGraph.append("\"" + n.getName() + "\"").append("[style=filled,fillcolor=\"aquamarine\"];");
		}
		for (Topic t : ROSComputationalGraph.getTopics()) {
			if (ROSComputationalGraph.getTopicSubscriptions(t.getName()).isEmpty()
					|| ROSComputationalGraph.getTopicPublishers(t.getName()).isEmpty()) {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"tan1\"];");
			} else {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"khaki1\"];");
			}
		}
		for (Node n : ROSComputationalGraph.getNodes()) {
			for (Publisher p : n.getPublishers()) {
				dotGraph.append("\"" + n.getName() + "\"").append(" -> ").append("\"" + p.getTopic().getName() + "\"");
				Boolean loop = false;
				Boolean perm = true;
				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Publisher _p :
				 * permGraph.getNodeByName(n.getName()).getPublishers()) { if
				 * (_p.getTopic().getName().equals(p.getTopic().getName())) {
				 * perm = true; break; } } }
				 */
				Set<Subscription> subs = ROSComputationalGraph.getTopicSubscriptions(p.getTopic().getName());
				for (Subscription s : subs) {
					if (s.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getMsgType()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getMsgType())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}

				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getMsgType()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getMsgType())
								.append("\",style=\"dotted\",penwidth=3];");
					}

				}
			}
			for (Subscription s : n.getSubscribers()) {
				dotGraph.append("\"" + s.getTopic().getName() + "\"").append(" -> ").append("\"" + n.getName() + "\"");
				Boolean loop = false;
				Boolean perm = true;
				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Subscription _s :
				 * permGraph.getNodeByName(n.getName()).getSubscribers()) { if
				 * (_s.getTopic().getName().equals(s.getTopic().getName())) {
				 * perm = true; break; } } }
				 */

				Set<Publisher> pubs = ROSComputationalGraph.getTopicPublishers("\"" + s.getTopic().getName() + "\"");
				for (Publisher p : pubs) {
					if (p.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}
				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\",style=\"dotted\",penwidth=3];");
					}
				}
			}
		}
		dotGraph.append("}");
		FileManager.WriteAction w = writer -> writer.write(dotGraph.toString());
		try {
			new FileManager(workDir + "/graph/").mkDotFile("graph", w);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public void dumpSecureGraph() throws Exception {
		StringBuilder dotGraph = new StringBuilder(
				"digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
		for (Node n : ROSComputationalGraph.getNodes()) {
			dotGraph.append("\"" + n.getName() + "\"").append("[style=filled,fillcolor=\"aquamarine\"];");
		}

		for (Topic t : ROSComputationalGraph.getTopics()) {
			if (ROSComputationalGraph.getTopicSubscriptions(t.getName()).isEmpty()
					|| ROSComputationalGraph.getTopicPublishers(t.getName()).isEmpty()) {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"tan1\"];");
			} else {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"khaki1\"];");
			}
		}

		for (Node n : ROSComputationalGraph.getNodes()) {
			for (Publisher p : n.getPublishers()) {
				dotGraph.append("\"" + n.getName() + "\"").append(" -> ").append("\"" + p.getTopic().getName() + "\"");
				Boolean loop = false;
				Grant permGrant = permissionsGrants.get(n.getName());

				Boolean perm = permGrant != null && permGrant.isPublisherAllowed(p.getTopic().getName());
				// PermissionsNode permissionsNode =
				// permissionsNodes.get(n.getName())
				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Publisher _p :
				 * permGraph.getNodeByName(n.getName()).getPublishers()) { if
				 * (_p.getTopic().getName().equals(p.getTopic().getName())) {
				 * perm = true; break; } } }
				 */
				Set<Subscription> subs = ROSComputationalGraph.getTopicSubscriptions(p.getTopic().getName());
				for (Subscription s : subs) {
					if (s.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getMsgType()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getMsgType())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}

				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getMsgType()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getMsgType())
								.append("\",style=\"dotted\",penwidth=3];");
					}

				}
			}
			for (Subscription s : n.getSubscribers()) {
				dotGraph.append("\"" + s.getTopic().getName() + "\"").append(" -> ").append("\"" + n.getName() + "\"");
				Boolean loop = false;
				Grant permGrant = permissionsGrants.get(n.getName());
				Boolean perm = permGrant != null && permGrant.isSubscriptionAllowed(s.getTopic().getName());

				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Subscription _s :
				 * permGraph.getNodeByName(n.getName()).getSubscribers()) { if
				 * (_s.getTopic().getName().equals(s.getTopic().getName())) {
				 * perm = true; break; } } }
				 */

				Set<Publisher> pubs = ROSComputationalGraph.getTopicPublishers(s.getTopic().getName());
				for (Publisher p : pubs) {
					if (p.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}
				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getMsgType()).append(", ")
								.append(s.getCallbackFunction()).append("\",style=\"dotted\",penwidth=3];");
					}
				}
			}
		}
		dotGraph.append("}");
		FileManager.WriteAction w = writer -> writer.write(dotGraph.toString());
		try {
			new FileManager(workDir + "/graph/").mkDotFile("graph-secure", w);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public void dumpInfo() throws Exception {
		StringBuilder res = new StringBuilder("*** ANALYSIS RESULTS ***\n");
		res.append("\t# NODES: " + ROSComputationalGraph.getNodes().size() + "\n");
		res.append("\t# TOPICS: " + ROSComputationalGraph.getTopics().size() + "\n\n");
		res.append("\t*** NODES ***:\n");
		if (ROSComputationalGraph.getNodes().isEmpty()) {
			res.append("\t\tNo Nodes found!\n");
		} else {
			for (Node n : ROSComputationalGraph.getNodes()) {
				res.append("\t\t* Name: " + n.getName() + "\n");
				res.append("\t\t\t* PUBLISHERS\n");
				for (Publisher p : n.getPublishers()) {
					res.append("\t\t\t\t* Topic: " + p.getTopic().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + p.getMsgType() + "\n");
				}
				res.append("\t\t\t* SUBSCRIPTIONS\n");
				for (Subscription s : n.getSubscribers()) {
					res.append("\t\t\t\t* Topic: " + s.getTopic().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + s.getMsgType() + "\n");
					res.append("\t\t\t\t\tCallback function: " + s.getCallbackFunction() + "\n");
				}
			}
		}
		res.append("\t*** TOPICS ***:\n");
		int pubNo = 0;
		int subNo = 0;
		if (ROSComputationalGraph.getTopics().isEmpty()) {
			res.append("\t\tNo Topics found!\n");
		} else {
			for (Topic t : ROSComputationalGraph.getTopics()) {
				res.append("\t\t* Name: " + t.getName() + "\n");
				res.append("\t\t\t* PUBLISHERS\n");
				for (Publisher p : ROSComputationalGraph.getTopicPublishers(t.getName())) {
					pubNo++;
					res.append("\t\t\t\t* Node: " + p.getNode().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + p.getMsgType() + "\n");
				}
				res.append("\t\t\t* SUBSCRIPTIONS\n");
				for (Subscription s : ROSComputationalGraph.getTopicSubscriptions(t.getName())) {
					subNo++;
					res.append("\t\t\t\t* Node: " + s.getNode().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + s.getMsgType() + "\n");
					res.append("\t\t\t\t\tCallback function: " + s.getCallbackFunction() + "\n");
				}
			}
		}
		res.append("\t# SUBSCRIBERS: " + subNo + "\n");
		res.append("\t# PUBLISHERS: " + pubNo + "\n");
		// ROSComputationalGraph.getTopicPublishers("ciao").iterator().next().getNode();

		FileManager.WriteAction w = writer -> writer.write(res.toString());
		new FileManager(workDir).mkOutputFile("output.txt", w);
	}

	public void dumpGraphUndAdjMatrix() throws Exception {
		StringBuilder res = new StringBuilder("§§§ IDs §§§\n");
		int id = 0;
		Map<Node, String> ids = new HashMap<>();
		for (Node n : getROSComputationalGraph().getNodes()) {
			ids.put(n, "" + id);
			res.append(id + " " + n.getName() + "\n");
			id++;
		}
		res.append("§§§ MATRIX §§§\n");
		StringBuilder matrix = new StringBuilder("");
		int vertices = ids.size();
		int edges = 0;
		Set<Pair<String, String>> verticesSet = new LinkedHashSet<>();
		for (Map.Entry<Node, String> entry : ids.entrySet()) {
			for (Topic t : entry.getKey().getAllNodeTopics()) {
				for (TopicUser tu : this.ROSComputationalGraph.getTopicUsers(t.getName())) {
					if (!tu.getNode().equals(entry.getKey())) {
						if (verticesSet.add(Pair.of(entry.getValue(), ids.get(tu.getNode())))) {
							matrix.append(entry.getValue() + " " + ids.get(tu.getNode()) + "\n");
							edges++;
						}
					}
				}
			}
		}
		res.append("#edges: " + edges + "\n");
		res.append("#vertices: " + vertices + "\n");
		res.append(matrix);
		FileManager.WriteAction w = writer -> writer.write(res.toString());
		new FileManager(workDir).mkOutputFile("und-graph-adj-matrix.txt", w);
	}

	public void dumpGraphDirAdjMatrix() throws Exception {
		StringBuilder res = new StringBuilder("§§§ IDs §§§\n");
		int id = 0;
		Map<Node, String> ids = new HashMap<>();
		for (Node n : getROSComputationalGraph().getNodes()) {
			ids.put(n, "" + id);
			res.append(id + " " + n.getName() + "\n");
			id++;
		}
		res.append("§§§ MATRIX §§§\n");
		StringBuilder matrix = new StringBuilder("");
		int vertices = ids.size();
		int edges = 0;
		Set<Pair<String, String>> verticesSet = new LinkedHashSet<>();
		for (Map.Entry<Node, String> entry : ids.entrySet()) {
			for (Topic t : entry.getKey().getPublisherTopics()) {
				for (TopicUser tu : this.ROSComputationalGraph.getTopicSubscriptions(t.getName())) {
					if (!tu.getNode().equals(entry.getKey())) {
						if (verticesSet.add(Pair.of(entry.getValue(), ids.get(tu.getNode())))) {
							matrix.append(entry.getValue() + " " + ids.get(tu.getNode()) + "\n");
							edges++;
						}
					}
				}
			}
		}
		if (matrix.length() > 0) {
			int lastnNewLine = matrix.length() - 1;
			matrix.replace(lastnNewLine, lastnNewLine + 1, "");
		}
		res.append("#edges: " + edges + "\n");
		res.append("#vertices: " + vertices + "\n");
		res.append(matrix);
		FileManager.WriteAction w = writer -> writer.write(res.toString());
		new FileManager(workDir).mkOutputFile("dir-graph-adj-matrix.txt", w);
	}
}
