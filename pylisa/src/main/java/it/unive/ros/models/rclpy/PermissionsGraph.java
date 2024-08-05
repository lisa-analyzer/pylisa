package it.unive.ros.models.rclpy;

import java.util.HashSet;
import java.util.Set;

public class PermissionsGraph {
	private Set<ROSNode> nodes;

	private Set<ROSTopic> topics;

	public PermissionsGraph() {
		this.nodes = new HashSet<>();
		this.topics = new HashSet<>();
	}

	public Set<ROSNode> getNodes() {
		return nodes;
	}

	public void addNode(
			ROSNode n) {
		nodes.add(n);
	}

	public ROSNode getNodeByName(
			String name) {
		for (ROSNode n : nodes) {
			if (n.getName().equals(name)) {
				return n;
			}
		}
		return null;
	}

	public ROSTopic addOrGetTopic(
			String name) {
		for (ROSTopic t : topics) {
			if (t.getName().equals(name)) {
				return t;
			}
		}
		ROSTopic t = new ROSTopic(name);
		topics.add(t);
		return t;
	}

	public Set<ROSTopicBasedNetworkEntity> getTopicUsers(
			String topicName) {
		Set<ROSTopicBasedNetworkEntity> topicUsers = new HashSet<>();
		for (ROSNode n : nodes) {
			for (ROSTopicBasedNetworkEntity tu : n.getAllNodeTopicsUsers()) {
				if (tu.getChannel().getName().equals(topicName)) {
					topicUsers.add(tu);
				}
			}
		}
		return topicUsers;
	}

	public Set<ROSTopicSubscription> getTopicSubscriptions(
			String topicName) {
		Set<ROSTopicSubscription> topicSubs = new HashSet<>();
		for (ROSNode n : nodes) {
			for (ROSTopicBasedNetworkEntity tu : n.getAllNodeTopicsUsers()) {
				if (tu.getChannel().getName().equals(topicName) && tu instanceof ROSTopicSubscription) {
					topicSubs.add((ROSTopicSubscription) tu);
				}
			}
		}
		return topicSubs;
	}

	public Set<ROSTopicPublisher> getTopicPublishers(
			String topicName) {
		Set<ROSTopicPublisher> topicPubs = new HashSet<>();
		for (ROSNode n : nodes) {
			for (ROSTopicBasedNetworkEntity tu : n.getAllNodeTopicsUsers()) {
				if (tu.getChannel().getName().equals(topicName) && tu instanceof ROSTopicPublisher) {
					topicPubs.add((ROSTopicPublisher) tu);
				}
			}
		}
		return topicPubs;
	}

	public Set<ROSTopic> getTopics() {
		return topics;
	}
}
