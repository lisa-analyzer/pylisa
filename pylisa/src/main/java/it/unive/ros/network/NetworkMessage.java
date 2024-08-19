package it.unive.ros.network;

public class NetworkMessage {
	NetworkEvent event;
	Object message;
	String type;

	public String getType() {
		return this.type;
	}

	public NetworkMessage(
			Object message,
			String type) {
		this.message = message;
		this.type = type;
	}

	public void setNetworkEvent(
			NetworkEvent event) {
		this.event = event;
	}

	public NetworkEvent getNetworkEvent() {
		return event;
	}

	public Object getMessage() {
		return message;
	}

}
