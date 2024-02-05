package it.unive.ros.application;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.ROSNetwork2;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.permissions.jaxb.Grant;
import it.unive.ros.permissions.jaxb.JAXBPermissionsHelpers;
import it.unive.ros.permissions.jaxb.PermissionsNode;
import jakarta.xml.bind.JAXBException;
import java.io.FileNotFoundException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RosApplicationBuilder {
	private String workDir = "ros-app-output";
	private final ArrayList<Program> programs = new ArrayList<>();
	ROSNetwork n = new ROSNetwork();
	private final ROSComputationGraphDumper rosGraphDumper = new ROSComputationGraphDumper(new RosComputationalGraph(), n);
	private final Map<String, Grant> permissionsGrants = new HashMap<>();
	private final List<ROSNodeBuilder> nodes = new ArrayList<>();
	private final List<String> fileNames = new ArrayList<>();

	public RosApplicationBuilder() {
	}

	public RosApplicationBuilder withNode(
			ROSNodeBuilder rosNodeBuilder)
			throws ROSNodeBuildException {
		nodes.add(rosNodeBuilder);
		// programs.add(rosNodeBuilder.getLiSAProgram());
		return this;
	}

	public RosApplicationBuilder withPermissions(
			String XMLfileName)
			throws ROSNodeBuildException,
			FileNotFoundException,
			JAXBException {
		PermissionsNode permsNode = JAXBPermissionsHelpers.load(XMLfileName);
		for (Grant g : permsNode.getPermissions().getGrant()) {
			permissionsGrants.putIfAbsent(g.getName(), g);
		}
		return this;
	}

	public RosApplicationBuilder withWorkDir(
			String workDir) {
		this.workDir = workDir;
		return this;
	}

	public ROSApplication build() throws ROSApplicationBuildException {
		// build nodes
		ArrayList<String> lisaOutputs = new ArrayList<>();
		try {

			for (ROSNodeBuilder node : nodes) {
				try {
					LiSA liSA = new LiSA(getLiSAConfiguration());
					Program p = node.getLiSAProgram();
					liSA.run(p);
					Field fileManagerLiSAField
							= LiSA.class.getDeclaredField("fileManager");
					fileManagerLiSAField.setAccessible(true);
					FileManager fileManager = (FileManager) fileManagerLiSAField.get(liSA);
					for (String name : fileManager.createdFiles()) {
						if (name.endsWith(".html")) {
							lisaOutputs.add(name);
						}
					}
					fileManagerLiSAField.setAccessible(false);
					var x = 3;
				} catch (Exception e) {
					System.out.println("[ERR] " + e.getMessage());
					throw new ROSApplicationBuildException(e);
				}
			}
			// liSA.run(programs.toArray(new Program[0]));
		} catch (Exception e) {
			throw new ROSApplicationBuildException(e);
		}
		return new ROSApplication(rosGraphDumper.getRosGraph(), permissionsGrants, rosGraphDumper.getNetwork(), lisaOutputs, workDir);
	}

	protected LiSAConfiguration getLiSAConfiguration() {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workDir;
		conf.serializeResults = false;
		conf.jsonOutput = false;
		conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.optimize = false;
		conf.semanticChecks.add(rosGraphDumper);
		PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap()
				.bottom();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
		conf.abstractState = new SimpleAbstractState<>(heap, domain, type);
		return conf;
	}
}
