package it.unive.ros.models.rclpy;

public class ServiceSubscription extends Subscription{
    public ServiceSubscription(Node node, Topic topic, String msgType, String callbackFunction) {
        super(node, topic, msgType, callbackFunction);
    }
}
