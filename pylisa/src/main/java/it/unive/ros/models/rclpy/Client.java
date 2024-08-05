package it.unive.ros.models.rclpy;

public class Client {
	private String name;
	private String srvType;

	public Client(
			String name,
			String srvType) {
		this.name = name;
		this.srvType = srvType;
	}

	public String getName() {
		return name;
	}

	public String getSrvType() {
		return srvType;
	}
}
