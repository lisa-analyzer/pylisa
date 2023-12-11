package it.unive.ros.models.rclpy;

public class ServicePublisher extends Publisher {

	public ServicePublisher(Node node, Topic topic, String msgType) {
		super(node, topic, msgType);
	}
}
