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
import it.unive.ros.network.*;

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

	public ROSTopicBasedNetworkEntity(String containerID, ROSTopic topic, String msgType, HeapExpression expr, Statement publisherStmt, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		super(null, topic, containerID, expr, publisherStmt, analysisState);
		this.msgType = msgType;
		this.processedEvents = new ArrayList<>();
	}


	public ROSNode getNode() {
		return getContainer();
	}

	public void setNode(ROSNode n) {
		this.setContainer(n);
	}

	public String getType() {
		return msgType;
	}

	@Override
	public List<NetworkEvent> getProcessedEvents() {
		return processedEvents;
	}

	public void addProcessedEvent(NetworkEvent ne) {
		processedEvents.add(ne);
	}
}
