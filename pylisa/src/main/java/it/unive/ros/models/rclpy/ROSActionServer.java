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

public class ROSActionServer extends ROSActionBasedNetworkEntity {


    public ROSActionServer(ROSNetwork network, ROSActionChannel channel, ROSNode node) {
        super(network, channel, node);
    }

    public ROSActionServer(ROSNetwork network, ROSActionChannel channel, String nodeID) {
        super(network, channel, nodeID);
    }

    public ROSActionServer(String containerID, ROSActionChannel channel, String msgType, HeapExpression expr, Statement publisherStmt, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
        super(containerID, channel, msgType, expr, publisherStmt, analysisState);
    }

    @Override
    public Set<ROSTopicBasedNetworkEntity> toTopicEntities() {
        Set<ROSTopicBasedNetworkEntity> topics = new HashSet<>();
        topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "/_action/status"), getType(), null, null, null));
        topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "/_action/feedback"), getType(), null, null, null));

        topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "/_action/send_goalReply"), getType(), null, null, null));
        topics.add(new ROSTopicSubscription(null, new ROSTopic(getChannel().getName() + "/_action/send_goalRequest"), getType(), null, null, null));

        topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "/_action/cancel_goalReply"), getType(), null, null, null));
        topics.add(new ROSTopicSubscription(null, new ROSTopic(getChannel().getName() + "/_action/cancel_goalRequest"), getType(), null, null, null));

        topics.add(new ROSTopicPublisher(null, new ROSTopic(getChannel().getName() + "/_action/get_resultReply"), getType(), null, null, null));
        topics.add(new ROSTopicSubscription(null, new ROSTopic(getChannel().getName() + "/_action/get_resultRequest"), getType(), null, null, null));

        return topics;
    }
}
