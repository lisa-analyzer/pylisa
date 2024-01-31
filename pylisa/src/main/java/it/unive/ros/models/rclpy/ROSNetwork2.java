package it.unive.ros.models.rclpy;

import it.unive.ros.network.NetworkEvent;
import it.unive.ros.network.NetworkMessage;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ROSNetwork2 {
/*
    private Set<ROSNode> networkEntityContainers;
    private Set<ROSCommunicationChannel> networkChannels;
    private List<NetworkEvent> networkEvents;

    private Set<ROSNetworkEntity<?>> networkEntities;

    public ROSNetwork2() {
        this.networkEntityContainers = new HashSet<>();
        this.networkChannels = new HashSet<>();
        this.networkEvents = new ArrayList<>();
        this.networkEntities = new HashSet<>();
    }
    public List<ROSTopic> getTopics() {
        ArrayList<ROSTopic> topics = new ArrayList<>();
        for (ROSCommunicationChannel channel : this.getNetworkChannels()) {
            if (channel instanceof ROSTopic) {
                topics.add((ROSTopic) channel);
            }
        }
        return topics;
    }

    public ROSTopic getTopic(String topicID) {
        for (ROSCommunicationChannel channel : this.getNetworkChannels()) {
            if (channel instanceof ROSTopic && channel.getID().equals(topicID)) {
               return (ROSTopic) channel;
            }
        }
        return null;
    }


    public ROSServiceChannel getServiceChannel(String topicID) {
        for (ROSCommunicationChannel channel : this.getNetworkChannels()) {
            if (channel instanceof ROSServiceChannel && channel.getID().equals(topicID)) {
                return (ROSServiceChannel) channel;
            }
        }
        return null;
    }
    public List<ROSNode> getTopicUserContainers(ROSTopic topic) {
        ArrayList<ROSNode> nodes = new ArrayList<>();
        for (ROSNode n : getNetworkEntityContainers()) {
           for (ROSTopicBasedNetworkEntity entity : n.getAllNodeTopicsUsers()) {
               if (entity.getTopic().equals(topic)) {
                    nodes.add(n);
                    break;
               }
           }
        }
        return nodes;
    }

    public Set<ROSNode> getNetworkEntityContainers() {
        return this.networkEntityContainers;
    }


    public Set<ROSCommunicationChannel> getNetworkChannels() {
        return this.networkChannels;
    }
    public ROSCommunicationChannel getNetworkChannel(String id) {
        for (ROSCommunicationChannel channel : this.networkChannels) {
            if (channel.getID().equals(id)) {
                return channel;
            }
        }
        return null;
    }

    public String toGraphviz(boolean secure) {
        StringBuilder dotGraph = new StringBuilder(
                "digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
        // Nodes
        for (ROSNode n : getNetworkEntityContainers()) {
            dotGraph.append("\"").append(n.getName()).append("\"").append("[style=filled,fillcolor=\"aquamarine\"];");
        }

        // Topic
        for (ROSTopic t : getTopics()) {
            if (getTopicUserContainers(t).isEmpty()) {
                dotGraph.append("\"").append(t.getName()).append("\"").append("[shape=box,style=filled,fillcolor=\"tan1\"];");
            } else {
                dotGraph.append("\"").append(t.getName()).append("\"").append("[shape=box,style=filled,fillcolor=\"khaki1\"];");
            }
        }

        // Publishers and Subscribers
        for (ROSNode n : getNetworkEntityContainers()) {
            for (ROSTopicPublisher p : n.getPublishers()) {
                dotGraph.append("\"").append(n.getName()).append("\"").append(" -> ").append("\"").append(p.getTopic().getName()).append("\"");
            }
            for (ROSTopicSubscription s : n.getSubscribers()) {
                dotGraph.append("\"").append(s.getTopic().getName()).append("\"").append(" -> ").append("\"").append(n.getName()).append("\"");
            }
        }
        dotGraph.append("}");
        return dotGraph.toString();
    }

    public ROSNetworkEntity<?> getNetworkEntity(String id) {
            for (ROSNetworkEntity<?> entity : this.networkEntities) {
                if (entity.getID().equals(id)) {
                    return entity;
                }
            }
            return null;
    }

    public List<ROSNetworkEntity<?>> getChannelNetworkEntities(String channelID) {
        List<ROSNetworkEntity<?>> result = new ArrayList<>();
        for (ROSNetworkEntity<?> ne : getNetworkEntities()) {
            if (ne.getChannel().getID().equals(channelID)) {
                result.add(ne);
            }
        }
        return result;
    }

    public Set<ROSNetworkEntity<?>> getNetworkEntities() {
        return networkEntities;
    }

    public void addEntityContainer(ROSNode node) throws Exception {
        if (this.getNetworkEntityContainer(node.getID()) == null) {
            this.networkEntityContainers.add(node);
            resolveFoundling(node);
        } else {
            throw new Exception("Duplicated NetworkEntityContainer with id " + node.getID());
        }
    }


    public void addNetworkEntity(ROSNetworkEntity<?> ne, String networkEntityContainerID) {
        ROSNode nec = getNetworkEntityContainer(networkEntityContainerID);
        if (nec != null) {
            ne.setContainer(nec);
            nec.addNetworkEntity(ne);
        }
        ne.setNetwork(this);
        this.networkEntities.add(ne);
    }

    public NetworkEvent createNetworkEvent(NetworkMessage message, ROSNetworkEntity<?> networkEntity) {
        NetworkEvent event = networkEntity.createNetworkEvent(message);
        addNetworkEvent(event);
        return event;
    }

    public void addNetworkEvent(NetworkEvent ne) {
        this.networkEvents.add(ne);
    }

    public void addNetworkChannel(ROSCommunicationChannel channel) throws Exception {
        if (this.getNetworkChannel(channel.getID()) == null) {
            this.networkChannels.add(channel);
        } else {
            throw new Exception("Duplicated NetworkChannel with id " + channel.getID());
        }
    }
    public void resolveFoundling(ROSNode container) {
        for (ROSNetworkEntity<?> ne : networkEntities) {
            if (ne.getContainer() == null && ne.getContainerID().equals(container.getID())) {
                ne.setContainer(container);
                container.addNetworkEntity(ne);
            }
        }
    }

    public ROSNode getNetworkEntityContainer(String id) {
        for (ROSNode container : this.networkEntityContainers) {
            if (container.getID().equals(id)) {
                return container;
            }
        }
        return null;
    }

    public void processEvents() throws Exception {
        while (!this.networkEvents.isEmpty()) {
            NetworkEvent e = this.networkEvents.get(0);
            e.process(this);
            this.networkEvents.remove(0);
        }
    }*/
}
