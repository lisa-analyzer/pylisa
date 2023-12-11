package it.unive.ros.models.rclpy;

import it.unive.lisa.interprocedural.ScopeId;
import java.util.HashSet;
import java.util.Set;

public class RosComputationalGraph {
	private Set<Node> nodes;

	private Set<Topic> topics;

	public RosComputationalGraph() {
		this.nodes = new HashSet<>();
		this.topics = new HashSet<>();
	}

	public Set<Node> getNodes() {
		return nodes;
	}

	public void addNode(Node n) {
		nodes.add(n);
	}

	public Node getNodeByName(String name) {
		for (Node n : nodes) {
			if (n.getName().equals(name)) {
				return n;
			}
		}
		return null;
	}

	public Node getNodeByScopeId(ScopeId scopeId) {
		for (Node n : nodes) {
			if (n.getScopeId().equals(scopeId)) {
				return n;
			}
		}
		return null;
	}

	public Topic addOrGetTopic(String name) {
		for (Topic t : topics) {
			if (t.getName().equals(name)) {
				return t;
			}
		}
		Topic t = new Topic(name);
		topics.add(t);
		return t;
	}

	public Set<TopicUser> getTopicUsers(String topicName) {
		Set<TopicUser> topicUsers = new HashSet<>();
		for (Node n : nodes) {
			for (TopicUser tu : n.getAllNodeTopicsUsers()) {
				if (tu.getTopic().getName().equals(topicName)) {
					topicUsers.add(tu);
				}
			}
		}
		return topicUsers;
	}

	public Set<Subscription> getTopicSubscriptions(String topicName) {
		Set<Subscription> topicSubs = new HashSet<>();
		for (Node n : nodes) {
			for (TopicUser tu : n.getAllNodeTopicsUsers()) {
				if (tu.getTopic().getName().equals(topicName) && tu instanceof Subscription) {
					topicSubs.add((Subscription) tu);
				}
			}
		}
		return topicSubs;
	}

	public Set<Publisher> getTopicPublishers(String topicName) {
		Set<Publisher> topicPubs = new HashSet<>();
		for (Node n : nodes) {
			for (TopicUser tu : n.getAllNodeTopicsUsers()) {
				if (tu.getTopic().getName().equals(topicName) && tu instanceof Publisher) {
					topicPubs.add((Publisher) tu);
				}
			}
		}
		return topicPubs;
	}

	public Set<Topic> getTopics() {
		return topics;
	}
}
