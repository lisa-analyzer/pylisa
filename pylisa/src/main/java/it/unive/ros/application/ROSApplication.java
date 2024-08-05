package it.unive.ros.application;

import com.ibm.icu.impl.Pair;
import it.unive.lisa.util.file.FileManager;
import it.unive.ros.models.rclpy.*;
import it.unive.ros.models.rclpy.ROSNode;
import it.unive.ros.permissions.jaxb.Grant;
import org.thymeleaf.TemplateEngine;
import org.thymeleaf.context.Context;
import org.thymeleaf.templatemode.TemplateMode;
import org.thymeleaf.templateresolver.ClassLoaderTemplateResolver;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDate;
import java.util.*;

public class ROSApplication {

	private String workDir;
	private final RosComputationalGraph ROSComputationalGraph;

	private final ROSNetwork rosNetwork;
	private Map<String, Grant> permissionsGrants = new HashMap<>();

	private ArrayList<String> outputFiles = new ArrayList<>();

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
		this.rosNetwork = new ROSNetwork();
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph,
			String workDir) {
		this.ROSComputationalGraph = ROSComputationalGraph;
		this.workDir = workDir;
		this.rosNetwork = new ROSNetwork();
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph,
			Map<String, Grant> permissionsGrants,
			ROSNetwork rosNetwork,
			String workDir) {
		this.ROSComputationalGraph = ROSComputationalGraph;
		this.workDir = workDir;
		this.permissionsGrants = permissionsGrants;
		this.rosNetwork = rosNetwork;
	}

	public ROSApplication(
			RosComputationalGraph ROSComputationalGraph,
			Map<String, Grant> permissionsGrants,
			ROSNetwork rosNetwork,
			ArrayList<String> outputFiles,
			String workDir) {
		this.ROSComputationalGraph = ROSComputationalGraph;
		this.workDir = workDir;
		this.permissionsGrants = permissionsGrants;
		this.rosNetwork = rosNetwork;
		this.outputFiles = outputFiles;
	}

	public ArrayList<String> getOutputFiles() {
		return outputFiles;
	}
	public RosComputationalGraph getROSComputationalGraph() {
		return ROSComputationalGraph;
	}

	public void dumpPermissions() throws Exception {
		for (ROSNode n : this.getRosNetwork().getNetworkEntityContainers()) {
			n.dumpPermissions(workDir);
		}
	}

	public void dumpResults() throws Exception {
		this.dumpGraph();
		Runtime rt = Runtime.getRuntime();
		rt.exec("dot  " + workDir + "/graph/graph.dot" + " -Tsvg -o " + workDir + "/graph/graph.svg");
		var resolver = new ClassLoaderTemplateResolver();
		resolver.setTemplateMode(TemplateMode.HTML);
		resolver.setCharacterEncoding("UTF-8");
		resolver.setPrefix("/templates/");
		resolver.setSuffix(".html");
		var context = new Context();
		context.setVariable("date", LocalDate.now().toString());
		context.setVariable("projectName", "simple_node");
		context.setVariable("graph", getROSComputationalGraph());
		context.setVariable("rosNetwork", getRosNetwork());
		context.setVariable("svgPath", "graph/graph.svg");
		context.setVariable("outputFiles", outputFiles);
		context.setVariable("xmlPermMainFolder", workDir + "/permissions/");
		var templateEngine = new TemplateEngine();
		System.out.println("PROCESSED EVENTS: " + getRosNetwork().getProcessedNetworkEvents().size());

		System.out.println("UNPROCESSED EVENTS: " + getRosNetwork().getNetworkEvents().size());
		templateEngine.setTemplateResolver(resolver);
		var result = templateEngine.process("index", context);
		FileWriter output = new FileWriter(workDir + "/report.html");
		// generate permissions .xml
		for (ROSNode n : rosNetwork.getNetworkEntityContainers()) {
			n.dumpPermissions(workDir);
		}
		for (ROSNode n : rosNetwork.getNetworkEntityContainers()) {
			n.dumpPolicy(workDir);
		}
		BufferedWriter Bout = new BufferedWriter(output);
		Bout.write(result);
		Bout.flush();
		Bout.close();
		output.close();
	}
	public void dumpGraph() throws Exception {
		/*StringBuilder dotGraph = new StringBuilder(
				"digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
		// DUMP Services
		int i = 0;

		for (ROSNode n : ROSComputationalGraph.getNodes()) {
			Set<Service> ss = n.getServices();
			for (Service s : ss) {

			}
			for (Service s : n.getServices()) {
				dotGraph.append("subgraph cluster_" + i
						+ " { style=filled;fillcolor=white;color=blue;penwidth=2;fontcolor=blue;label=\"" + s.getName()
						+ "\";").append("\"" + s.getName() + "Reply\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];").append("\"" + s.getName() + "Request\"")
						.append("[shape=box,style=filled];}");
				i++;
			}
			for (Action a : n.getActions()) {
				dotGraph.append("subgraph cluster_" + i
						+ " { style=filled;fillcolor=white;color=seagreen3;penwidth=2;fontcolor=seagreen3;label=\"" + a.getName()
						+ "\";").append("\"" + a.getName() + "/_action/send_goalReply\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/send_goalRequest\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/get_resultRequest\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/get_resultReply\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/cancel_goalRequest\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/cancel_goalReply\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/feedback\"")
						.append("[shape=box,style=filled,fillcolor=\"tan1\"];")
						.append("\"" + a.getName() + "/_action/status\"")
						.append("[shape=box,style=filled];}");
				i++;
			}
			dotGraph.append("\"" + n.getName() + "\"").append("[style=filled,fillcolor=\"aquamarine\"];");
		}
		for (ROSTopic t : ROSComputationalGraph.getTopics()) {
			if (ROSComputationalGraph.getTopicSubscriptions(t.getName()).isEmpty()
					|| ROSComputationalGraph.getTopicPublishers(t.getName()).isEmpty()) {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"tan1\"];");
			} else {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"khaki1\"];");
			}
		}
		for (ROSNode n : ROSComputationalGraph.getNodes()) {
			for (ROSTopicPublisher p : n.getPublishers()) {
				dotGraph.append("\"" + n.getName() + "\"").append(" -> ").append("\"" + p.getChannel().getName() + "\"");
				Boolean loop = false;
				Boolean perm = true;
				Set<ROSTopicSubscription> subs = ROSComputationalGraph.getTopicSubscriptions(p.getChannel().getName());
				for (ROSTopicSubscription s : subs) {
					if (s.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getType()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getType())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}

				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getType()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getType())
								.append("\",style=\"dotted\",penwidth=3];");
					}

				}
			}
			for (ROSTopicSubscription s : n.getSubscribers()) {
				dotGraph.append("\"" + s.getChannel().getName() + "\"").append(" -> ").append("\"" + n.getName() + "\"");
				Boolean loop = false;
				Boolean perm = true;

				Set<ROSTopicPublisher> pubs = ROSComputationalGraph.getTopicPublishers("\"" + s.getChannel().getName() + "\"");
				for (ROSTopicPublisher p : pubs) {
					if (p.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getType()).append(", ")
								.append(s.getCallbackFunction()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getType()).append(", ")
								.append(s.getCallbackFunction())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}
				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getType()).append(", ")
								.append(s.getCallbackFunction()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getType()).append(", ")
								.append(s.getCallbackFunction()).append("\",style=\"dotted\",penwidth=3];");
					}
				}
			}
		}
		dotGraph.append("}");*/
		FileManager.WriteAction w = writer -> writer.write(rosNetwork.toGraphviz(false));
		try {
			new FileManager(workDir + "/graph/").mkDotFile("graph", w);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public void dumpSecureGraph() throws Exception {
		StringBuilder dotGraph = new StringBuilder(
				"digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
		for (ROSNode n : ROSComputationalGraph.getNodes()) {
			dotGraph.append("\"" + n.getName() + "\"").append("[style=filled,fillcolor=\"aquamarine\"];");
		}

		for (ROSTopic t : ROSComputationalGraph.getTopics()) {
			if (ROSComputationalGraph.getTopicSubscriptions(t.getName()).isEmpty()
					|| ROSComputationalGraph.getTopicPublishers(t.getName()).isEmpty()) {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"tan1\"];");
			} else {
				dotGraph.append("\"" + t.getName() + "\"").append("[shape=box,style=filled,fillcolor=\"khaki1\"];");
			}
		}

		for (ROSNode n : ROSComputationalGraph.getNodes()) {
			for (ROSTopicPublisher p : n.getPublishers()) {
				dotGraph.append("\"" + n.getName() + "\"").append(" -> ").append("\"" + p.getChannel().getName() + "\"");
				Boolean loop = false;
				Grant permGrant = permissionsGrants.get(n.getName());

				Boolean perm = permGrant != null && permGrant.isPublisherAllowed(p.getChannel().getName());
				// PermissionsNode permissionsNode =
				// permissionsNodes.get(n.getName())
				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Publisher _p :
				 * permGraph.getNodeByName(n.getName()).getPublishers()) { if
				 * (_p.getChannel().getName().equals(p.getChannel().getName())) {
				 * perm = true; break; } } }
				 */
				Set<ROSTopicSubscription> subs = ROSComputationalGraph.getTopicSubscriptions(p.getChannel().getName());
				for (ROSTopicSubscription s : subs) {
					if (s.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getType()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getType())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}

				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(p.getType()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(p.getType())
								.append("\",style=\"dotted\",penwidth=3];");
					}

				}
			}
			for (ROSTopicSubscription s : n.getSubscribers()) {
				dotGraph.append("\"" + s.getChannel().getName() + "\"").append(" -> ").append("\"" + n.getName() + "\"");
				Boolean loop = false;
				Grant permGrant = permissionsGrants.get(n.getName());
				Boolean perm = permGrant != null && permGrant.isSubscriptionAllowed(s.getChannel().getName());

				/*
				 * Node _n = permGraph.getNodeByName(n.getName()); if (_n !=
				 * null) { for (Subscription _s :
				 * permGraph.getNodeByName(n.getName()).getSubscribers()) { if
				 * (_s.getChannel().getName().equals(s.getChannel().getName())) {
				 * perm = true; break; } } }
				 */

				Set<ROSTopicPublisher> pubs = ROSComputationalGraph.getTopicPublishers(s.getChannel().getName());
				for (ROSTopicPublisher p : pubs) {
					if (p.getNode().equals(n)) {
						loop = true;
						break;
					}
				}
				if (loop) {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getType()).append(", ")
								.append(s.getCallbackFunction()).append("\",color=\"red\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getType()).append(", ")
								.append(s.getCallbackFunction())
								.append("\",color=\"red\",style=\"dotted\",penwidth=3];");
					}
				} else {
					if (perm) {
						dotGraph.append("[shape=box,label=\"").append(s.getType()).append(", ")
								.append(s.getCallbackFunction()).append("\"];");
					} else {
						dotGraph.append("[shape=box,label=\"⚠️").append(s.getType()).append(", ")
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
			for (ROSNode n : ROSComputationalGraph.getNodes()) {
				res.append("\t\t* Name: " + n.getName() + "\n");
				res.append("\t\t\t* PUBLISHERS\n");
				for (ROSTopicPublisher p : n.getPublishers()) {
					res.append("\t\t\t\t* Topic: " + p.getChannel().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + p.getType() + "\n");
				}
				res.append("\t\t\t* SUBSCRIPTIONS\n");
				for (ROSTopicSubscription s : n.getSubscribers()) {
					res.append("\t\t\t\t* Topic: " + s.getChannel().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + s.getType() + "\n");
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
			for (ROSTopic t : ROSComputationalGraph.getTopics()) {
				res.append("\t\t* Name: " + t.getName() + "\n");
				res.append("\t\t\t* PUBLISHERS\n");
				for (ROSTopicPublisher p : ROSComputationalGraph.getTopicPublishers(t.getName())) {
					pubNo++;
					res.append("\t\t\t\t* Node: " + p.getNode().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + p.getType() + "\n");
				}
				res.append("\t\t\t* SUBSCRIPTIONS\n");
				for (ROSTopicSubscription s : ROSComputationalGraph.getTopicSubscriptions(t.getName())) {
					subNo++;
					res.append("\t\t\t\t* Node: " + s.getNode().getName() + "\n");
					res.append("\t\t\t\t\tMsg Type: " + s.getType() + "\n");
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
		Map<ROSNode, String> ids = new HashMap<>();
		for (ROSNode n : getROSComputationalGraph().getNodes()) {
			ids.put(n, "" + id);
			res.append(id + " " + n.getName() + "\n");
			id++;
		}
		res.append("§§§ MATRIX §§§\n");
		StringBuilder matrix = new StringBuilder();
		int vertices = ids.size();
		int edges = 0;
		Set<Pair<String, String>> verticesSet = new LinkedHashSet<>();
		for (Map.Entry<ROSNode, String> entry : ids.entrySet()) {
			for (ROSTopic t : entry.getKey().getAllNodeTopics()) {
				for (ROSTopicBasedNetworkEntity tu : this.ROSComputationalGraph.getTopicUsers(t.getName())) {
					if (!tu.getNode().equals(entry.getKey())) {
						if (verticesSet.add(Pair.of(entry.getValue(), ids.get(tu.getNode())))) {
							matrix.append(entry.getValue()).append(" ").append(ids.get(tu.getNode())).append("\n");
							edges++;
						}
					}
				}
			}
		}
		res.append("#edges: ").append(edges).append("\n");
		res.append("#vertices: ").append(vertices).append("\n");
		res.append(matrix);
		FileManager.WriteAction w = writer -> writer.write(res.toString());
		new FileManager(workDir).mkOutputFile("und-graph-adj-matrix.txt", w);
	}

	public void dumpGraphDirAdjMatrix() throws Exception {
		StringBuilder res = new StringBuilder("§§§ IDs §§§\n");
		int id = 0;
		Map<ROSNode, String> ids = new HashMap<>();
		for (ROSNode n : getROSComputationalGraph().getNodes()) {
			ids.put(n, "" + id);
			res.append(id).append(" ").append(n.getName()).append("\n");
			id++;
		}
		res.append("§§§ MATRIX §§§\n");
		StringBuilder matrix = new StringBuilder();
		int vertices = ids.size();
		int edges = 0;
		Set<Pair<String, String>> verticesSet = new LinkedHashSet<>();
		for (Map.Entry<ROSNode, String> entry : ids.entrySet()) {
			for (ROSTopic t : entry.getKey().getPublisherTopics()) {
				for (ROSTopicBasedNetworkEntity tu : this.ROSComputationalGraph.getTopicSubscriptions(t.getName())) {
					if (!tu.getNode().equals(entry.getKey())) {
						if (verticesSet.add(Pair.of(entry.getValue(), ids.get(tu.getNode())))) {
							matrix.append(entry.getValue()).append(" ").append(ids.get(tu.getNode())).append("\n");
							edges++;
						}
					}
				}
			}
		}
		if (!matrix.isEmpty()) {
			int lastnNewLine = matrix.length() - 1;
			matrix.replace(lastnNewLine, lastnNewLine + 1, "");
		}
		res.append("#edges: " + edges + "\n");
		res.append("#vertices: " + vertices + "\n");
		res.append(matrix);
		FileManager.WriteAction w = writer -> writer.write(res.toString());
		new FileManager(workDir).mkOutputFile("dir-graph-adj-matrix.txt", w);
	}

	public ROSNetwork getRosNetwork() {return rosNetwork;}
}
