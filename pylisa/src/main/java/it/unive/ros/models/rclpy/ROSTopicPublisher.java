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
import it.unive.ros.network.NetworkEntityType;
import it.unive.ros.network.NetworkEvent;
import it.unive.ros.network.NetworkMessage;

public class ROSTopicPublisher extends ROSTopicBasedNetworkEntity {

	public ROSTopicPublisher(
			ROSNode node,
			ROSTopic topic,
			String msgType) {
		super(node, topic, msgType);
	}

	public ROSTopicPublisher(
			String containerID,
			ROSTopic topic,
			String msgType,
			Statement publisherStmt,
			HeapExpression expr,
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<InferredTypes>>> analysisState) {
		super(containerID, topic, msgType, expr, publisherStmt, analysisState);
	}

	@Override
	public NetworkEvent createNetworkEvent(
			NetworkMessage message) {
		NetworkEvent event = new PublishMessageEvent(message);
		message.setNetworkEvent(event);
		event.setInitiator(this);
		return event;
	}

	@Override
	public NetworkEntityType getNetworkEntityType() {
		return NetworkEntityType.WRITER;
	}

	@Override
	public void processMessage(
			NetworkMessage message) {
		// A publisher can't receive a message!
		return;
	}

}