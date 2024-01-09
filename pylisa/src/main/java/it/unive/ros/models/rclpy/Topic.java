package it.unive.ros.models.rclpy;

import it.unive.ros.network.NetworkChannel;

public class Topic implements NetworkChannel {
	private String ID;

	public Topic(
			String name) {
		this.ID = name;
	}

	public String getName() {
		return this.ID;
	}

	@Override
	public String getID() {
		return this.ID;
	}
}
