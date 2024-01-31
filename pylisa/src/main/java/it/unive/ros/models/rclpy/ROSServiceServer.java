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

import java.util.HashSet;
import java.util.Set;

public class ROSServiceServer extends ROSServiceBasedNetworkEntity {
	private String name;

	private String callback;


	public ROSServiceServer(ROSNetwork network, ROSServiceChannel channel, ROSNode node) {
		super(network, channel, node);
	}

	public ROSServiceServer(ROSNetwork network, ROSServiceChannel channel, String nodeID) {
		super(network, channel, nodeID);
	}

	@Override
	public Set<ROSTopicBasedNetworkEntity> toTopicEntities() {
		Set<ROSTopicBasedNetworkEntity> topics = new HashSet<>();
		topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "Reply", getChannel().isSystem(), ROSTopicType.SERVICE_RESPONSE), getType(), null, null, null));
		topics.add(new ROSTopicSubscription(null, new ROSTopic(getChannel().getName() + "Request", getChannel().isSystem(), ROSTopicType.SERVICE_REQUEST), getType(), null, null, null));
		return topics;
	}

	public ROSServiceServer(String containerID, ROSServiceChannel channel, String msgType, Statement publisherStmt, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		super(containerID, channel, msgType, expr, publisherStmt, analysisState);
	}


	public String getName() {
		return name;
	}


	public String getCallback() {
		return callback;
	}
}
