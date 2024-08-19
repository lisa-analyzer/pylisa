package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.NetworkEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

abstract class ROSActionBasedNetworkEntity extends ROSNetworkEntity<ROSActionChannel> {

	private String msgType;

	private List<NetworkEvent> processedEvents = new ArrayList<>();

	public ROSActionBasedNetworkEntity(
			ROSNetwork network,
			ROSActionChannel channel,
			ROSNode node) {
		super(network, channel, node);
	}

	public ROSActionBasedNetworkEntity(
			ROSNetwork network,
			ROSActionChannel channel,
			String nodeID) {
		super(network, channel, nodeID);
	}

	public ROSActionBasedNetworkEntity(
			String containerID,
			ROSActionChannel channel,
			String msgType,
			HeapExpression expr,
			Statement publisherStmt,
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<InferredTypes>>> analysisState) {
		super(null, channel, containerID, expr, publisherStmt, analysisState);
		this.msgType = msgType;
	}

	public ROSNode getNode() {
		return getContainer();
	}

	public void setNode(
			ROSNode n) {
		this.setContainer(n);
	}

	public String getType() {
		return msgType;
	}

	@Override
	public List<NetworkEvent> getProcessedEvents() {
		return processedEvents;
	}

	public void addProcessedEvent(
			NetworkEvent ne) {
		processedEvents.add(ne);
	}

	public abstract Set<ROSTopicBasedNetworkEntity> toTopicEntities();
}
