package it.unive.ros.models.rclpy;

import it.unive.ros.network.NetworkChannel;

public abstract class ROSCommunicationChannel implements NetworkChannel {
	private String ID;
	private Boolean system = false;
	private Boolean avoidROSNamespaceConventions = false;

	public ROSCommunicationChannel(
			String ID) {
		this.ID = ID;
	}

	public ROSCommunicationChannel(
			String ID,
			Boolean system) {
		this.ID = ID;
		this.system = system;
	}

	public ROSCommunicationChannel(
			String id,
			boolean system,
			boolean avoidRosNamespaceConventions) {
		this(id, system);
		this.avoidROSNamespaceConventions = avoidRosNamespaceConventions;
	}

	@Override
	public String getID() {
		return ID;
	}

	public String getName() {
		return ID;
	}

	public Boolean isAvoidRosNamespaceConventions() {
		return avoidROSNamespaceConventions;
	}

	public Boolean isSystem() {
		return system;
	}
}
