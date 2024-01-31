package it.unive.ros.models.rclpy;

import it.unive.ros.network.Network;

import java.util.HashSet;
import java.util.Set;

public class ROSNetwork extends Network<ROSNode, ROSNetworkEntity<? extends ROSCommunicationChannel>, ROSCommunicationChannel> {

    @Override
    public void addEntityContainer(ROSNode container) throws Exception {
        super.addEntityContainer(container);
        if (container.getEnableParameterServices()) {
            ROSServiceChannel serviceChannel = new ROSServiceChannel(container.getURI() + "/describe_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/get_parameter_types", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/get_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/list_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/set_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/set_parameters_atomically", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container), container.getID());
        }
        if (container.getEnableRosout()) {
            ROSTopic topic = getTopic("/rosout");
            if (topic == null) {
                topic = new ROSTopic("/rosout", true);
                addNetworkChannel(topic);
            }
            addNetworkEntity(new ROSTopicPublisher(container, topic, "rcl_interfaces/msg/ParameterEvent"), container.getID());
        }
        ROSTopic topic = getTopic("/parameter_events");
        if (topic == null) {
            topic = new ROSTopic("/parameter_events", true);
            addNetworkChannel(topic);
        }
        addNetworkEntity(new ROSTopicPublisher(container, topic, "rcl_interfaces/msg/ParameterEvent"), container.getID());
        topic = getTopic("ros_discovery_info");
        if (topic == null) {
            topic = new ROSTopic("ros_discovery_info", true, true);
            addNetworkChannel(topic);
        }
        addNetworkEntity(new ROSTopicPublisher(container, topic, "rmw_dds_common/msg/ParticipantEntitiesInfo"), container.getID());
        addNetworkEntity(new ROSTopicSubscription(container, topic, "rmw_dds_common/msg/ParticipantEntitiesInfo", null), container.getID());

    }

    public ROSTopic getTopic(String topicName) {
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSTopic && channel.getID().equals(topicName)) {
                return (ROSTopic) channel;
            }
        }
        return null;
    }

    public ROSActionChannel getActionChannel(String actionName) {
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSActionChannel && channel.getID().equals(actionName)) {
                return (ROSActionChannel) channel;
            }
        }
        return null;
    }
    @Override
    public void addNetworkEntity(ROSNetworkEntity<? extends ROSCommunicationChannel> ne, String networkEntityContainerID) {
        ROSNode node = getNetworkEntityContainer(networkEntityContainerID);
        if (node != null) {
            ne.setContainer(node);
            node.addNetworkEntity(ne);
        }
        ne.setNetwork(this);
        this.getNetworkEntities().add(ne);

    }

    public ROSServiceChannel getServiceChannel(String serviceName) {
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSServiceChannel && channel.getID().equals(serviceName)) {
                return (ROSServiceChannel) channel;
            }
        }
        return null;
    }
    public String toMermaid() {
        StringBuilder mermaid = new StringBuilder(
                "graph TD\n");
        // Nodes
        for (ROSNode n : getNetworkEntityContainers()) {
            mermaid.append(n.getURI()).append("\n");
            mermaid.append("style ").append(n.getURI()).append(" fill:limegreen,stroke:#333,stroke-width:2px\n");
        }

        // Topic
        for (ROSTopic t : this.getTopics()) {
            if (!t.isSystem()) {
                if (this.getChannelUserContainers(t).isEmpty()) {
                    mermaid.append(t.getName()).append("[/").append(t.getName()).append("/]\n");
                    mermaid.append("style ").append(t.getName()).append(" fill:gold,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(t.getName()).append("[/").append(t.getName()).append("/]\n");
                    mermaid.append("style ").append(t.getName()).append(" fill:gold,stroke:#333,stroke-width:2px\n");
                }
            }
        }
        for (ROSServiceChannel s : this.getServices()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:lightskyblue,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:lightskyblue,stroke:#333,stroke-width:2px\n");
                }
            }
        }

        for (ROSActionChannel s : this.getActions()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:orchid1,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:orchid1,stroke:#333,stroke-width:2px\n");
                }
            }
        }

        // Publishers and Subscribers
        for (ROSNode n : getNetworkEntityContainers()) {
            for (ROSTopicPublisher p : n.getPublishers()) {
                if (!p.getChannel().isSystem()) {
                    mermaid.append(n.getURI()).append(" --> ").append(p.getChannel().getName()).append("\n");
                }
            }
            for (ROSTopicSubscription s : n.getSubscribers()) {
                if (!s.getChannel().isSystem()) {
                    mermaid.append(s.getChannel().getName()).append(" --> ").append(n.getURI()).append("\n");
                }
            }
            for (ROSServiceServer ss : n.getServiceServers()) {
                if (!ss.getChannel().isSystem()) {
                    mermaid.append(ss.getChannel().getName()).append(" --> ").append(n.getURI()).append("\n");
                }
            }
            for (ROSServiceClient sc : n.getServiceClients()) {
                if (!sc.getChannel().isSystem()) {
                    mermaid.append(n.getURI()).append(" --> ").append(sc.getChannel().getName()).append("\n");
                }
            }
            for (ROSActionServer as : n.getActionServers()) {
                if (!as.getChannel().isSystem()) {
                    mermaid.append(as.getChannel().getName()).append(" --> ").append(n.getURI()).append("\n");
                }
            }
            for (ROSActionClient ac : n.getActionClients()) {
                if (!ac.getChannel().isSystem()) {
                    mermaid.append(n.getURI()).append(" --> ").append(ac.getChannel().getName()).append("\n");
                }
            }
        }
        return mermaid.toString();
    }
    public String toGraphviz(boolean secure) {
        StringBuilder dotGraph = new StringBuilder(
                "digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
        // Nodes
        for (ROSNode n : getNetworkEntityContainers()) {
            dotGraph.append("\"").append(n.getURI()).append("\"").append("[style=filled,fillcolor=\"limegreen\"];");
        }

        // Topic
        for (ROSTopic t : this.getTopics()) {
            if (!t.isSystem()) {
                if (this.getChannelUserContainers(t).isEmpty()) {
                    dotGraph.append("\"").append(t.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"gold\"];");
                } else {
                    dotGraph.append("\"").append(t.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"gold\"];");
                }
            }
        }
        for (ROSServiceChannel s : this.getServices()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                } else {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                }
            }
        }

        for (ROSActionChannel s : this.getActions()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"orchid1\"];");
                } else {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"orchid1\"];");
                }
            }
        }

        // Publishers and Subscribers
        for (ROSNode n : getNetworkEntityContainers()) {
            for (ROSTopicPublisher p : n.getPublishers()) {
                if (!p.getChannel().isSystem()) {
                    dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(p.getChannel().getName()).append("\"");
                }
            }
            for (ROSTopicSubscription s : n.getSubscribers()) {
                if (!s.getChannel().isSystem()) {
                    dotGraph.append("\"").append(s.getChannel().getName()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                }
                }
            for (ROSServiceServer ss : n.getServiceServers()) {
                if (!ss.getChannel().isSystem()) {
                    dotGraph.append("\"").append(ss.getChannel().getName()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                }
            }
            for (ROSServiceClient sc : n.getServiceClients()) {
                if (!sc.getChannel().isSystem()) {
                    dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(sc.getChannel().getName()).append("\"");
                }
            }
            for (ROSActionServer as : n.getActionServers()) {
                if (!as.getChannel().isSystem()) {
                    dotGraph.append("\"").append(as.getChannel().getName()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                }
            }
            for (ROSActionClient ac : n.getActionClients()) {
                if (!ac.getChannel().isSystem()) {
                    dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(ac.getChannel().getName()).append("\"");
                }
            }
        }
        dotGraph.append("}");
        return dotGraph.toString();
    }

    private Set<ROSTopic> getTopics() {
        Set<ROSTopic> topics = new HashSet<>();
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSTopic) {
                topics.add((ROSTopic) channel);
            }
        }
        return topics;
    }

    private Set<ROSServiceChannel> getServices() {
        Set<ROSServiceChannel> services = new HashSet<>();
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSServiceChannel) {
                services.add((ROSServiceChannel) channel);
            }
        }
        return services;
    }

    private Set<ROSActionChannel> getActions() {
        Set<ROSActionChannel> services = new HashSet<>();
        for (ROSCommunicationChannel channel : getNetworkChannels()) {
            if (channel instanceof ROSActionChannel) {
                services.add((ROSActionChannel) channel);
            }
        }
        return services;
    }
    public Set<ROSNode> getChannelUserContainers(ROSCommunicationChannel channel) {
        Set<ROSNode> nodes = new HashSet<>();
        for (ROSNode n : getNetworkEntityContainers()) {
            for (ROSTopicBasedNetworkEntity entity : n.getAllNodeTopicsUsers()) {
                if (entity.getChannel().equals(channel)) {
                    nodes.add(n);
                    break;
                }
            }
        }
        return nodes;
    }
}
