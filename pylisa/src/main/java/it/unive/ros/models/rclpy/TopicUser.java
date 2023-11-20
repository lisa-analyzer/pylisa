package it.unive.ros.models.rclpy;

public abstract class TopicUser {
    private Topic topic;
    private String msgType;
    private Node node;
    public TopicUser(Node node, Topic topic, String msgType) {
        this.node = node;
        this.topic = topic;
        this.msgType = msgType;
    }

    public Topic getTopic() {
        return topic;
    }
    public Node getNode() {
        return node;
    }

    public String getMsgType() {
        return msgType;
    }
}
