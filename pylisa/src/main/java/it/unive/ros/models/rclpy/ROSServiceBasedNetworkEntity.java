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

public abstract class ROSServiceBasedNetworkEntity extends ROSNetworkEntity<ROSServiceChannel>{
    String srvType;

    private List<NetworkEvent> processedEvents = new ArrayList<>();

    public ROSServiceBasedNetworkEntity(ROSNetwork network, ROSServiceChannel channel, ROSNode node) {
        super(network, channel, node);
    }

    public ROSServiceBasedNetworkEntity(ROSNetwork network, ROSServiceChannel channel, String nodeID) {
        super(network, channel, nodeID);
    }

    public ROSServiceBasedNetworkEntity(String containerID, ROSServiceChannel channel, String srvType, HeapExpression expr, Statement publisherStmt, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
        super(null, channel, containerID, expr, publisherStmt, analysisState);
        this.srvType = srvType;
    }
    @Override
    public String getType() {
        return this.srvType;
    }

    @Override
    public List<NetworkEvent> getProcessedEvents() {
        return processedEvents;
    }

    public abstract Set<ROSTopicBasedNetworkEntity> toTopicEntities();

    public void addProcessedEvent(NetworkEvent ne) {
        processedEvents.add(ne);
    }
}
