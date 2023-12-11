package it.unive.ros.models.rclpy;

import java.util.ArrayList;
import java.util.List;

public class PermissionSatisfabilityNodeResult {
	Node node;
	List<Subscription> invalidSubscriptions = new ArrayList<>();
	List<Publisher> invalidPublishers = new ArrayList<>();

	public PermissionSatisfabilityNodeResult(Node n) {
		this.node = n;
	}

	public boolean isInvalid() {
		return !invalidSubscriptions.isEmpty() || !invalidPublishers.isEmpty();
	}

	public void addInvalidPublisher(Publisher p) {
		this.invalidPublishers.add(p);
	}

	public void addInvalidSubscription(Subscription s) {
		this.invalidSubscriptions.add(s);
	}
}
