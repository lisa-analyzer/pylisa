package it.unive.ros.models.rclpy;

import it.unive.ros.network.*;

import java.util.ArrayList;
import java.util.List;

public abstract class TopicUser implements NetworkEntity {
	private Topic topic;
	private String containerID;

	private Node container;

	private String msgType;
	private Node node;

	private List<NetworkEvent> processedEvents;
	@Override
	public void setContainer(NetworkEntityContainer nec) {
		this.container = (Node) nec;
	}

	@Override
	public String getContainerID() {
		return containerID;
	}

	@Override
	public void setContainerID(String containerID) {
		this.containerID = containerID;
	}

	@Override
	public void setNetwork(Network n) {
		this.network = n;
	}


	private Network network;
	public TopicUser(
			Node node,
			Topic topic,
			String msgType) {
		this.node = node;
		this.topic = topic;
		this.msgType = msgType;
		this.processedEvents = new ArrayList<>();
	}

	public TopicUser(
			String containerID,
			Topic topic,
			String msgType) {
		this.containerID = containerID;
		this.topic = topic;
		this.msgType = msgType;
		this.processedEvents = new ArrayList<>();
	}
	public Topic getTopic() {
		return topic;
	}

	public Node getNode() {
		return node;
	}

	public void setNode(Node n) {
		this.node = node;
	}

	public String getMsgType() {
		return msgType;
	}



	@Override
	public NetworkEntityContainer getContainer() {
		return this.container;
	}

	@Override
	public NetworkChannel getChannel() {
		return this.topic;
	}


	@Override
	public List<NetworkEvent> getProcessedEvents() {
		return processedEvents;
	}

	public void addProcessedEvent(NetworkEvent ne) {
		processedEvents.add(ne);
	}
}
