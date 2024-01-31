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
import java.util.List;

public abstract class ROSNetworkEntity<Channel extends ROSCommunicationChannel> implements NetworkEntity<ROSNode, Channel>{

    private ROSLisaAnalysis rosLisaAnalysis;

    private ROSNetwork network;
    private ROSNode node;

    private String nodeID;
    
    private final Channel channel;
    public ROSNetworkEntity(ROSNetwork network, Channel channel, ROSNode node) {
        this.network = network;
        this.channel = channel;
        this.node = node;
        this.nodeID = node.getID();
    }

    public ROSNetworkEntity(ROSNetwork network, Channel channel, String nodeID) {
        this.network = network;
        this.channel = channel;
        this.nodeID = nodeID;
    }

    public ROSNetworkEntity(ROSNode node, Channel topic, String nodeID, HeapExpression expr, Statement publisherStmt, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
        this.node = node;
        this.nodeID = nodeID;
        this.channel = topic;
        this.rosLisaAnalysis = new ROSLisaAnalysis(expr, publisherStmt, analysisState);
    }

    public NetworkEvent createNetworkEvent(NetworkMessage networkMessage) {
        return null;
    }


    public ROSNode getContainer() {
        return node;
    }


    public void setContainer(ROSNode node) {
        this.node = node;
    }


    public String getContainerID() {
        return nodeID;
    }


    public void setContainerID(String nodeID) {
        this.nodeID = nodeID;
    }

    public void setNetwork(ROSNetwork n) {
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


    public void processMessage(NetworkMessage message) throws Exception {

    }

    public ROSLisaAnalysis getRosLisaAnalysis() {
        return rosLisaAnalysis;
    }

    public String getID() {
        return rosLisaAnalysis.getSymbolicExpression().getCodeLocation().toString();
    }

    public abstract String getType();
}
