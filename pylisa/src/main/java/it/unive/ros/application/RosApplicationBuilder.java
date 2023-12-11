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
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.permissions.jaxb.Grant;
import it.unive.ros.permissions.jaxb.JAXBPermissionsHelpers;
import it.unive.ros.permissions.jaxb.PermissionsNode;
import jakarta.xml.bind.JAXBException;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class RosApplicationBuilder {
	private String workDir = "ros-app-output";
	private final ArrayList<Program> programs = new ArrayList<>();
	private final ROSComputationGraphDumper rosGraphDumper = new ROSComputationGraphDumper(new RosComputationalGraph());
	private final Map<String, Grant> permissionsGrants = new HashMap<>();

	public RosApplicationBuilder() {
	}

	public RosApplicationBuilder withNode(
			ROSNodeBuilder rosNodeBuilder)
			throws ROSNodeBuildException {
		programs.add(rosNodeBuilder.getLiSAProgram());
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
		try {
			LiSA liSA = new LiSA(getLiSAConfiguration());
			liSA.run(programs.toArray(new Program[0]));
		} catch (Exception e) {
			throw new ROSApplicationBuildException(e);
		}
		return new ROSApplication(rosGraphDumper.getRosGraph(), permissionsGrants, workDir);
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
