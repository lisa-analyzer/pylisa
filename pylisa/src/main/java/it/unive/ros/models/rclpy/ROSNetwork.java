package it.unive.ros.models.rclpy;

import it.unive.ros.network.Network;
import it.unive.ros.network.NetworkEntity;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ROSNetwork extends Network<ROSNode, ROSNetworkEntity<? extends ROSCommunicationChannel>, ROSCommunicationChannel> {

    @Override
    public void addEntityContainer(ROSNode container) throws Exception {
        super.addEntityContainer(container);
        if (container.getEnableParameterServices()) {
            ROSServiceChannel serviceChannel = new ROSServiceChannel(container.getURI() + "/describe_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/DescribeParameters"), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/get_parameter_types", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/GetParameterTypes"), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/get_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/GetParameters"), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/list_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/ListParameters"), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/set_parameters", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/SetParameters"), container.getID());
            serviceChannel = new ROSServiceChannel(container.getURI() + "/set_parameters_atomically", true);
            addNetworkChannel(serviceChannel);
            addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "rcl_interfaces/srv/SetParametersAtomically"), container.getID());
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

        ROSServiceChannel serviceChannel = new ROSServiceChannel(container.getURI() + "/get_type_description", true);
        addNetworkChannel(serviceChannel);
        addNetworkEntity(new ROSServiceServer(this, serviceChannel, container, "type_description_interfaces/srv/GetTypeDescription"), container.getID());

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
            mermaid.append(n.getURI()).append(":::wide\n");
            mermaid.append("style ").append(n.getURI()).append(" fill:limegreen,stroke:#333,stroke-width:2px\n");
        }

        // Topic
        for (ROSTopic t : this.getTopics()) {
            if (!t.isSystem()) {
                if (this.getChannelUserContainers(t).isEmpty()) {
                    mermaid.append(t.getName()).append("[/").append(t.getName()).append("/]:::wide\n");
                    mermaid.append("style ").append(t.getName()).append(" fill:gold,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(t.getName()).append("[/").append(t.getName()).append("/]:::wide\n");
                    mermaid.append("style ").append(t.getName()).append(" fill:gold,stroke:#333,stroke-width:2px\n");
                }
            }
        }
        for (ROSServiceChannel s : this.getServices()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]:::wide\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:lightskyblue,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]:::wide\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:lightskyblue,stroke:#333,stroke-width:2px\n");
                }
            }
        }

        for (ROSActionChannel s : this.getActions()) {
            if (!s.isSystem()) {
                if (this.getChannelUserContainers(s).isEmpty()) {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]:::wide\n");
                    mermaid.append("style ").append(s.getName()).append(" fill:orchid1,stroke:#333,stroke-width:2px\n");
                } else {
                    mermaid.append(s.getName()).append("[/").append(s.getName()).append("/]:::wide\n");
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
        mermaid.append("classDef wide padding:50px\n");
        return mermaid.toString();
    }
    public String toGraphviz(boolean secure) {
        int i = 0; // subgraph_cluster_index

        StringBuilder dotGraph = new StringBuilder(
                "digraph rosgraph {graph [pad=\"1\", nodesep=\"2\", rankdir=\"BT\", ranksep=\"2\"];");
        // Nodes
        for (ROSNode n : getNetworkEntityContainers()) {
            dotGraph.append("\"").append(n.getURI()).append("\"").append("[style=filled,fillcolor=\"limegreen\"];");
        }

        // Topic
        for (ROSTopic t : this.getTopics()) {
            if (!t.isSystem()) {
                if (this.getChannelUserContainers(t).isEmpty()) {
                    dotGraph.append("\"").append(t.getID()).append("\"").append("[shape=box,style=filled,fillcolor=\"gold\"];");
                } else {
                    dotGraph.append("\"").append(t.getID()).append("\"").append("[shape=box,style=filled,fillcolor=\"gold\"];");
                }
            }
        }
        for (ROSServiceChannel s : this.getServices()) {
            List<String> topicNames = new ArrayList<>();
            if (!s.isSystem()) {
                /*if (this.getChannelUserContainers(s).isEmpty()) {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                } else {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                }*/
                dotGraph.append("subgraph cluster_" + i)
                        .append(" { style=filled;fillcolor=\"lightskyblue\";penwidth=2;label=\"")
                        .append(s.getName())
                        .append("\";");
                i+= 1;
                for (NetworkEntity n : this.getNetworkEntities()) {
                    if (n instanceof ROSServiceClient || n instanceof ROSServiceServer) {
                        ROSServiceBasedNetworkEntity service = (ROSServiceBasedNetworkEntity) n;
                        if (service.getChannel().equals(s) && !topicNames.contains(service.getChannel().getID())) {
                            Set<ROSTopicBasedNetworkEntity> topics = service.toTopicEntities();
                            for (ROSTopicBasedNetworkEntity t : topics) {
                                dotGraph.append("\"")
                                        .append(t.getChannel().getID())
                                        .append("\"")
                                        .append("[shape=box,style=filled,fillcolor=\"gold\"];");
                                topicNames.add(t.getChannel().getID());
                            }
                        }
                    }
                }
                dotGraph.append("}");
                //dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
            }
        }

        for (ROSActionChannel s : this.getActions()) {
            List<String> topicNames = new ArrayList<>();
            if (!s.isSystem()) {
                /*if (this.getChannelUserContainers(s).isEmpty()) {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                } else {
                    dotGraph.append("\"").append(s.getName()).append("\"").append("[shape=parallelogram,style=filled,fillcolor=\"lightskyblue\"];");
                }*/
                dotGraph.append("subgraph cluster_" + i)
                        .append(" { style=filled;fillcolor=\"orchid1\";penwidth=2;label=\"")
                        .append(s.getName())
                        .append("\";");
                i += 1;
                for (NetworkEntity n : this.getNetworkEntities()) {
                    if (n instanceof ROSActionClient || n instanceof ROSActionServer) {
                        ROSActionBasedNetworkEntity action = (ROSActionBasedNetworkEntity) n;
                        if (action.getChannel().equals(s) && !topicNames.contains(action.getChannel().getID())) {
                            Set<ROSTopicBasedNetworkEntity> topics = action.toTopicEntities();
                            for (ROSTopicBasedNetworkEntity t : topics) {
                                dotGraph.append("\"")
                                        .append(t.getChannel().getID())
                                        .append("\"")
                                        .append("[shape=box,style=filled,fillcolor=\"gold\"];");
                                topicNames.add(t.getChannel().getID());
                            }
                        }
                    }
                }
                dotGraph.append("}");
            }
        }

        // Publishers and Subscribers
        for (ROSNode n : getNetworkEntityContainers()) {
            for (ROSTopicPublisher p : n.getPublishers()) {
                if (!p.getChannel().isSystem()) {
                    dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(p.getChannel().getID()).append("\"");
                }
            }
            for (ROSTopicSubscription s : n.getSubscribers()) {
                if (!s.getChannel().isSystem()) {
                    dotGraph.append("\"").append(s.getChannel().getID()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                }
                }
            for (ROSServiceServer ss : n.getServiceServers()) {
                if (!ss.getChannel().isSystem()) {
                    for (ROSTopicBasedNetworkEntity t : ss.toTopicEntities()) {
                        if (t instanceof ROSTopicPublisher) {
                            dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(t.getChannel().getID()).append("\"");
                        } else {
                            // is a Subscription
                            dotGraph.append("\"").append(t.getChannel().getID()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                        }
                    }
                }
            }
            for (ROSServiceClient sc : n.getServiceClients()) {
                if (!sc.getChannel().isSystem()) {
                    for (ROSTopicBasedNetworkEntity t : sc.toTopicEntities()) {
                        if (t instanceof ROSTopicPublisher) {
                            dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(t.getChannel().getID()).append("\"");
                        } else {
                            // is a Subscription
                            dotGraph.append("\"").append(t.getChannel().getID()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                        }
                    }
                }
            }
            for (ROSActionServer as : n.getActionServers()) {
                if (!as.getChannel().isSystem()) {
                    for (ROSTopicBasedNetworkEntity t : as.toTopicEntities()) {
                        if (t instanceof ROSTopicPublisher) {
                            dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(t.getChannel().getID()).append("\"");
                        } else {
                            // is a Subscription
                            dotGraph.append("\"").append(t.getChannel().getID()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                        }
                    }
                }
            }
            for (ROSActionClient ac : n.getActionClients()) {
                if (!ac.getChannel().isSystem()) {
                    for (ROSTopicBasedNetworkEntity t : ac.toTopicEntities()) {
                        if (t instanceof ROSTopicPublisher) {
                            dotGraph.append("\"").append(n.getURI()).append("\"").append(" -> ").append("\"").append(t.getChannel().getID()).append("\"");
                        } else {
                            // is a Subscription
                            dotGraph.append("\"").append(t.getChannel().getID()).append("\"").append(" -> ").append("\"").append(n.getURI()).append("\"");
                        }
                    }
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
