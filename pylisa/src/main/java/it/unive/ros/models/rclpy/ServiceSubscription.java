package it.unive.ros.models.rclpy;

public class ServiceSubscription extends ROSTopicSubscription {
	public ServiceSubscription(
			ROSNode node,
			ROSTopic topic,
			String msgType,
			String callbackFunction) {
		super(node, topic, msgType, null);
	}
}
