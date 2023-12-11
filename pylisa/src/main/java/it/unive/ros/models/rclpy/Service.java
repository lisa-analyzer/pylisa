package it.unive.ros.models.rclpy;

public class Service {
	private String name;
	private String srvType;

	private String callback;

	public Service(String name, String srvType, String callback) {
		this.name = name;
		this.srvType = srvType;
		this.callback = callback;
	}

	public String getName() {
		return name;
	}

	public String srvType() {
		return srvType;
	}

	public String getCallback() {
		return callback;
	}
}
