package it.unive.ros.network;

import java.util.List;

public interface NetworkEntity<Container extends NetworkEntityContainer, Channel extends NetworkChannel> {

    NetworkEvent createNetworkEvent(NetworkMessage networkMessage);
    Container getContainer();
    void setContainer(Container container);


    String getContainerID();

    void setContainerID(String containerID);
    Channel getChannel();

    NetworkEntityType getNetworkEntityType();

    List<NetworkEvent> getProcessedEvents();

    void processMessage(NetworkMessage message) throws Exception;
    String getID();

}