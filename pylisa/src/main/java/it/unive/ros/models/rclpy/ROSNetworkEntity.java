package it.unive.ros.models.rclpy;

import java.util.List;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.NetworkEntity;
import it.unive.ros.network.NetworkEntityType;
import it.unive.ros.network.NetworkEvent;
import it.unive.ros.network.NetworkMessage;

public abstract class ROSNetworkEntity<
		Channel extends ROSCommunicationChannel> implements NetworkEntity<ROSNode, Channel> {

	private ROSLisaAnalysis rosLisaAnalysis;

	private ROSNetwork network;
	private ROSNode node;

	private String nodeID;

	private final Channel channel;

	public ROSNetworkEntity(
			ROSNetwork network,
			Channel channel,
			ROSNode node) {
		this.network = network;
		this.channel = channel;
		this.node = node;
		this.nodeID = node.getID();
	}

	public ROSNetworkEntity(
			ROSNetwork network,
			Channel channel,
			String nodeID) {
		this.network = network;
		this.channel = channel;
		this.nodeID = nodeID;
	}

	public ROSNetworkEntity(
			ROSNode node,
			Channel topic,
			String nodeID,
			HeapExpression expr,
			Statement publisherStmt,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState) {
		this.node = node;
		this.nodeID = nodeID;
		this.channel = topic;
		this.rosLisaAnalysis = new ROSLisaAnalysis(expr, publisherStmt, analysisState);
	}

	public NetworkEvent createNetworkEvent(
			NetworkMessage networkMessage) {
		return null;
	}

	public ROSNode getContainer() {
		return node;
	}

	public void setContainer(
			ROSNode node) {
		this.node = node;
	}

	public String getContainerID() {
		return nodeID;
	}

	public void setContainerID(
			String nodeID) {
		this.nodeID = nodeID;
	}

	public void setNetwork(
			ROSNetwork n) {
		this.network = n;
	}

	public Channel getChannel() {
		return channel;
	}

	public NetworkEntityType getNetworkEntityType() {
		return null;
	}

	public List<NetworkEvent> getProcessedEvents() {
		return null;
	}

	public void processMessage(
			NetworkMessage message)
			throws Exception {

	}

	public ROSLisaAnalysis getRosLisaAnalysis() {
		return rosLisaAnalysis;
	}

	public String getID() {
		return rosLisaAnalysis.getSymbolicExpression().getCodeLocation().toString();
	}

	public abstract String getType();
}
