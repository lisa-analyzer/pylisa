package it.unive.ros.models.pika;

import it.unive.ros.network.*;

import java.util.List;

public class PikaConsumer implements NetworkEntity {
    NetworkEntityContainer nec;

    String containerID;
    private Network network;
    List<NetworkEvent> processedEvents;

    NetworkChannel channel;

    public PikaConsumer(String necID, NetworkChannel channel) {
        this.channel = channel;
        this.containerID = necID;
    }
    @Override
    public NetworkEvent createNetworkEvent(NetworkMessage networkMessage) {
        return null;
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
    public void processMessage(NetworkMessage message) {
        System.out.println("[" + this.getContainer().getName() + "::" + this.getChannel().getID() + "] RECEIVED [" + message.getMessage() + "] FROM: [" + message.getNetworkEvent().getInitiator().getContainer().getName() + "]");

    }

    @Override
    public String getID() {
        return null;
    }
}
