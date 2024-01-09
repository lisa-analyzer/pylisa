package it.unive.ros.network;

import it.unive.lisa.analysis.SemanticException;

import java.util.List;

public interface NetworkEntity {

    NetworkEvent createNetworkEvent(NetworkMessage networkMessage);
    NetworkEntityContainer getContainer();
    void setContainer(NetworkEntityContainer nec);
    String getContainerID();

    void setContainerID(String containerID);
    void setNetwork(Network n);
    NetworkChannel getChannel();

    NetworkEntityType getNetworkEntityType();

    List<NetworkEvent> getProcessedEvents();

    void processMessage(NetworkMessage message) throws SemanticException;
    String getID();
}