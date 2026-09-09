package it.unive.ros.models.rclpy;

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
import it.unive.ros.network.NetworkEvent;
import java.util.ArrayList;
import java.util.List;

public abstract class ROSTopicBasedNetworkEntity extends ROSNetworkEntity<ROSTopic> {

	private String msgType;

	private List<NetworkEvent> processedEvents;

	public ROSTopicBasedNetworkEntity(
			ROSNode node,
			ROSTopic topic,
			String msgType) {
		super(null, topic, node);
		this.msgType = msgType;
		this.processedEvents = new ArrayList<>();
	}

	public ROSTopicBasedNetworkEntity(
			String containerID,
			ROSTopic topic,
			String msgType,
			HeapExpression expr,
			Statement publisherStmt,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState) {
		super(null, topic, containerID, expr, publisherStmt, analysisState);
		this.msgType = msgType;
		this.processedEvents = new ArrayList<>();
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
}
