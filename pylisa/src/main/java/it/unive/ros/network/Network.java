package it.unive.ros.network;

import it.unive.lisa.analysis.SemanticException;
import it.unive.ros.models.rclpy.ROSCommunicationChannel;
import it.unive.ros.models.rclpy.ROSNode;
import it.unive.ros.models.rclpy.ROSTopic;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Network<Container extends NetworkEntityContainer, Entity extends NetworkEntity, Channel extends NetworkChannel> {
    private Set<Container> networkEntityContainers;
    private Set<Channel> networkChannels;
    private List<NetworkEvent> networkEvents;

    private List<NetworkEvent> processedNetworkEvents = new ArrayList<>();
    private List<Entity> networkEntities;
    public Network() {
        this.networkChannels = new HashSet<>();
        this.networkEntityContainers = new HashSet<>();
        this.networkEvents = new ArrayList<>();
        this.networkEntities = new ArrayList<>();
    }
    public List<NetworkEvent> getNetworkEvents() {
        return networkEvents;
    }
    public void addEntityContainer(Container container) throws Exception {
        if (this.getNetworkEntityContainer(container.getID()) == null) {
            this.networkEntityContainers.add(container);
            resolveFoundling(container);
        } else {
            throw new Exception("Duplicated NetworkEntityContainer with id " + container.getID());
        }
    }

    public Container getNetworkEntityContainer(String id) {
        for (Container container : this.networkEntityContainers) {
            if (container.getID().equals(id)) {
                return container;
            }
        }
        return null;
    }

    public Entity getNetworkEntity(String id) {
        for (Entity entity : this.networkEntities) {
            if (entity.getID().equals(id)) {
                return entity;
            }
        }
        return null;
    }

    public List<Entity> getNetworkEntities() {
        return networkEntities;
    }

    public Set<Container> getNetworkEntityContainers() {
        return networkEntityContainers;
    }
    public List<Entity> getChannelNetworkEntities(String channelID) {
        List<Entity> result = new ArrayList<>();
        for (Entity ne : getNetworkEntities()) {
            if (ne.getChannel().getID().equals(channelID)) {
                result.add(ne);
            }
        }
        return result;
    }
    public void addNetworkChannel(Channel channel) throws Exception {
        if (this.getNetworkChannel(channel.getID()) == null) {
            this.networkChannels.add(channel);
        } else {
            throw new Exception("Duplicated NetworkChannel with id " + channel.getID());
        }
    }

    public Channel getNetworkChannel(String id) {
        for (Channel channel : this.networkChannels) {
            if (channel.getID().equals(id)) {
                return channel;
            }
        }
        return null;
    }

    public Set<Channel> getNetworkChannels() {
        return this.networkChannels;
    }


    public NetworkEvent createNetworkEvent(NetworkMessage message, Entity networkEntity) {
        NetworkEvent event = networkEntity.createNetworkEvent(message);
        addNetworkEvent(event);
        return event;
    }
    public void processEvents() throws Exception {
         while (!this.networkEvents.isEmpty()) {
            NetworkEvent e = this.networkEvents.get(0);
            e.process(this);
            this.networkEvents.remove(0);
            this.processedNetworkEvents.add(e);
        }
    }
    public List<NetworkEvent> getProcessedNetworkEvents() {
        return networkEvents;
    }
    public void addNetworkEntity(Entity ne, String networkEntityContainerID) {
        Container nec = getNetworkEntityContainer(networkEntityContainerID);
        if (nec != null) {
            ne.setContainer(nec);
            nec.addNetworkEntity(ne);
        }
        this.networkEntities.add(ne);
    }

    public void addNetworkEvent(NetworkEvent ne) {
        this.networkEvents.add(ne);
    }
    public void resolveFoundling(Container container) {
        for (Entity ne : networkEntities) {
            if (ne.getContainer() == null && ne.getContainerID().equals(container.getID())) {
                ne.setContainer(container);
                container.addNetworkEntity(ne);
            }
        }
    }

}
