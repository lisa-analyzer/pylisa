package it.unive.ros.models.pika;

import it.unive.ros.models.rclpy.PublishMessageEvent;
import it.unive.ros.network.*;

import java.util.List;

public class PikaPublisher implements NetworkEntity {
    NetworkEntityContainer nec;

    String containerID;
    private Network network;
    List<NetworkEvent> processedEvents;

    String routingKey;
    NetworkChannel channel;

    public PikaPublisher(NetworkEntityContainer nec, NetworkChannel channel) {
        this.channel = channel;
        this.containerID = nec.getID();
        this.nec = nec;
    }

    public PikaPublisher(NetworkEntityContainer nec, String routingKey,  NetworkChannel channel) {
        this.channel = channel;
        this.containerID = nec.getID();
        this.routingKey = routingKey;
        this.nec = nec;
    }
    public PikaPublisher(String necID, String routingKey, NetworkChannel channel) {
        this.channel = channel;
        this.routingKey = routingKey;
        this.containerID = necID;
    }

    public String getRoutingKey() {
        return routingKey;
    }
    @Override
    public NetworkEvent createNetworkEvent(NetworkMessage networkMessage) {
        NetworkEvent event = new PublishMessageEvent(networkMessage);
        networkMessage.setNetworkEvent(event);
        event.setInitiator(this);
        return event;
    }

    @Override
    public NetworkEntityContainer getContainer() {
        return nec;
    }

    @Override
    public void setContainer(NetworkEntityContainer nec) {
        this.nec = nec;
    }

    @Override
    public String getContainerID() {
        return containerID;
    }

    @Override
    public void setContainerID(String containerID) {
        this.containerID = containerID;
    }

    @Override
    public void setNetwork(Network n) {
        this.network = n;
    }

    @Override
    public NetworkChannel getChannel() {
        return channel;
    }

    @Override
    public NetworkEntityType getNetworkEntityType() {
        return NetworkEntityType.WRITER;
    }

    @Override
    public List<NetworkEvent> getProcessedEvents() {
        return processedEvents;
    }

    @Override
    public void processMessage(NetworkMessage message) {}

    @Override
    public String getID() {
        return null;
    }
}
