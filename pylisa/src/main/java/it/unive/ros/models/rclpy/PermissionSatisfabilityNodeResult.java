package it.unive.ros.models.rclpy;

import java.util.ArrayList;
import java.util.List;

public class PermissionSatisfabilityNodeResult {
	ROSNode node;
	List<ROSTopicSubscription> invalidSubscriptions = new ArrayList<>();
	List<ROSTopicPublisher> invalidPublishers = new ArrayList<>();

	public PermissionSatisfabilityNodeResult(
			ROSNode n) {
		this.node = n;
	}

	public boolean isInvalid() {
		return !invalidSubscriptions.isEmpty() || !invalidPublishers.isEmpty();
	}

	public void addInvalidPublisher(
			ROSTopicPublisher p) {
		this.invalidPublishers.add(p);
	}

	public void addInvalidSubscription(
			ROSTopicSubscription s) {
		this.invalidSubscriptions.add(s);
	}
}
