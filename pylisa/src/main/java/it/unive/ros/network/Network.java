package it.unive.ros.network;

import it.unive.lisa.analysis.SemanticException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Network {
    private Set<NetworkEntityContainer> networkEntityContainers;
    private Set<NetworkChannel> networkChannels;
    private List<NetworkEvent> networkEvents;

    private List<NetworkEntity> networkEntities;
    public Network() {
        this.networkChannels = new HashSet<>();
        this.networkEntityContainers = new HashSet<>();
        this.networkEvents = new ArrayList<>();
        this.networkEntities = new ArrayList<>();
    }
    public List<NetworkEvent> getNetworkEvents() {
        return networkEvents;
    }
    public void addEntityContainer(NetworkEntityContainer container) throws Exception {
        if (this.getNetworkEntityContainer(container.getID()) == null) {
            this.networkEntityContainers.add(container);
            resolveFoundling(container);
        } else {
            throw new Exception("Duplicated NetworkEntityContainer with id " + container.getID());
        }
    }

    public NetworkEntityContainer getNetworkEntityContainer(String id) {
        for (NetworkEntityContainer container : this.networkEntityContainers) {
            if (container.getID().equals(id)) {
                return container;
            }
        }
        return null;
    }

    public NetworkEntity getNetworkEntity(String id) {
        for (NetworkEntity entity : this.networkEntities) {
            if (entity.getID().equals(id)) {
                return entity;
            }
        }
        return null;
    }

    public List<NetworkEntity> getNetworkEntities() {
        return networkEntities;
    }

    public Set<NetworkEntityContainer> getNetworkEntityContainers() {
        return networkEntityContainers;
    }
    public List<NetworkEntity> getChannelNetworkEntities(String channelID) {
        List<NetworkEntity> result = new ArrayList<>();
        for (NetworkEntity ne : getNetworkEntities()) {
            if (ne.getChannel().getID().equals(channelID)) {
                result.add(ne);
            }
        }
        return result;
    }
    public void addNetworkChannel(NetworkChannel channel) throws Exception {
        if (this.getNetworkChannel(channel.getID()) == null) {
            this.networkChannels.add(channel);
        } else {
            throw new Exception("Duplicated NetworkChannel with id " + channel.getID());
        }
    }

    public NetworkChannel getNetworkChannel(String id) {
        for (NetworkChannel channel : this.networkChannels) {
            if (channel.getID().equals(id)) {
                return channel;
            }
        }
        return null;
    }

    public NetworkEvent createNetworkEvent(NetworkMessage message, NetworkEntity networkEntity) {
        NetworkEvent event = networkEntity.createNetworkEvent(message);
        addNetworkEvent(event);
        return event;
    }
    public void processEvents() throws Exception {
        while (!this.networkEvents.isEmpty()) {
            NetworkEvent e = this.networkEvents.get(0);
            e.process(this);
            this.networkEvents.remove(0);
        }
    }

    public void forwardMessage() {

    }


    public void addNetworkEntity(NetworkEntity ne, String networkEntityContainerID) {
        NetworkEntityContainer nec = getNetworkEntityContainer(networkEntityContainerID);
        if (nec != null) {
            ne.setContainer(nec);
            nec.addNetworkEntity(ne);
        }
        ne.setNetwork(this);
        this.networkEntities.add(ne);
    }

    public void addNetworkEvent(NetworkEvent ne) {
        this.networkEvents.add(ne);
    }
    public void resolveFoundling(NetworkEntityContainer nec) {
        for (NetworkEntity ne : networkEntities) {
            if (ne.getContainer() == null && ne.getContainerID().equals(nec.getID())) {
                ne.setContainer(nec);
                nec.addNetworkEntity(ne);
            }
        }
    }

}
