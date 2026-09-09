package it.unive.ros.models.rclpy;

import java.util.HashSet;
import java.util.Set;

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

public class ROSServiceClient extends ROSServiceBasedNetworkEntity {
	private String name;

	private String callback;

	public ROSServiceClient(
			ROSNetwork network,
			ROSServiceChannel channel,
			ROSNode node) {
		super(network, channel, node);
	}

	public ROSServiceClient(
			ROSNetwork network,
			ROSServiceChannel channel,
			String nodeID) {
		super(network, channel, nodeID);
	}

	@Override
	public Set<ROSTopicBasedNetworkEntity> toTopicEntities() {
		Set<ROSTopicBasedNetworkEntity> topics = new HashSet<>();
		topics.add(new ROSTopicSubscription(
				null, new ROSTopic(getChannel().getName() + "Reply", getChannel().isSystem(),
						ROSTopicType.SERVICE_RESPONSE, getChannel().isAvoidRosNamespaceConventions()),
				getType(), null, null, null));
		topics.add(new ROSTopicPublisher(
				null, new ROSTopic(getChannel().getName() + "Request", getChannel().isSystem(),
						ROSTopicType.SERVICE_REQUEST, getChannel().isAvoidRosNamespaceConventions()),
				getType(), null, null, null));
		return topics;
	}

	public ROSServiceClient(
			String containerID,
			ROSServiceChannel channel,
			String msgType,
			Statement publisherStmt,
			HeapExpression expr,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState) {
		super(containerID, channel, msgType, expr, publisherStmt, analysisState);
	}

	public String getName() {
		return name;
	}

	public String getCallback() {
		return callback;
	}
}
