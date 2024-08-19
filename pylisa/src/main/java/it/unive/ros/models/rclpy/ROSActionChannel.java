package it.unive.ros.models.rclpy;

public class ROSActionChannel extends ROSCommunicationChannel {
	public ROSActionChannel(
			String ID) {
		super(ID);
	}

	public ROSActionChannel(
			String ID,
			boolean avoidRosNamespaceConventions) {
		super(ID, false, avoidRosNamespaceConventions);
	}
}
