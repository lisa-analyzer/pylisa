package it.unive.ros.models.rclpy;


public class ROSTopic extends ROSCommunicationChannel {
	private ROSTopicType topicType = ROSTopicType.DEFAULT;
	private boolean avoidROSNamespaceConventions = false;
	public ROSTopic(
			String name) {
		super(name);
	}

	public ROSTopic(String name, boolean system) {
		super(name, system);
	}

	public ROSTopic(String name, boolean system, boolean avoidROSNamespaceConventions) {
		super(name, system);
		this.avoidROSNamespaceConventions = avoidROSNamespaceConventions;
	}

	public ROSTopic(String name, boolean system, ROSTopicType topicType) {
		super(name, system);
		this.topicType = topicType;
	}
	public ROSTopic(String name, boolean system, ROSTopicType topicType, Boolean avoidROSNamespaceConventions) {
		super(name, system);
		this.topicType = topicType;
		this.avoidROSNamespaceConventions = avoidROSNamespaceConventions;
	}
	public String getName() {
		return this.getID();
	}

	public boolean isAvoidROSNamespaceConventions() {
		return avoidROSNamespaceConventions;
	}

	public ROSTopicType getTopicType() {
		return topicType;
	}

	public String getDDSPrefix() {
		if (avoidROSNamespaceConventions) {
			return "";
		}
        return switch (topicType) {
            case ACTION -> "ra";
            case SERVICE_REQUEST -> "rq";
            case SERVICE_RESPONSE -> "rr";
            default -> "rt";
        };
	}
}
