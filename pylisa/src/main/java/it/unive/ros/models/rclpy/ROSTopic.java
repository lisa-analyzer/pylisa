package it.unive.ros.models.rclpy;


public class ROSTopic extends ROSCommunicationChannel {
	private ROSTopicType topicType = ROSTopicType.DEFAULT;
	public ROSTopic(
			String name) {
		super(name);
	}

	public ROSTopic(String name, boolean system) {
		super(name, system);
	}

	public ROSTopic(String name, boolean system, boolean avoidROSNamespaceConventions) {
		super(name, system, avoidROSNamespaceConventions);
	}

	public ROSTopic(String name, boolean system, ROSTopicType topicType) {
		super(name, system);
		this.topicType = topicType;
	}
	public ROSTopic(String name, boolean system, ROSTopicType topicType, Boolean avoidROSNamespaceConventions) {
		super(name, system, avoidROSNamespaceConventions);
		this.topicType = topicType;
	}
	public String getName() {
		return super.getID();
	}
	public String getID() {
		return this.getDDSPrefix() + super.getID();
	}

	public ROSTopicType getTopicType() {
		return topicType;
	}

	public String getDDSPrefix() {
		if (isAvoidRosNamespaceConventions()) {
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
