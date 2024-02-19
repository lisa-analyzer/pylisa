package it.unive.ros.models.rclpy;

public class ROSServiceChannel extends ROSCommunicationChannel{

    public ROSServiceChannel(String ID) {
        super(ID);
    }


    public ROSServiceChannel(String id, boolean system) {
        super(id, system);
    }

    public ROSServiceChannel(String id, boolean system, boolean avoidRosNamespaceConventions) {
        super(id, system, avoidRosNamespaceConventions);
    }
    public String getName() {
        return super.getID();
    }
    public String getID() {
        return this.getDDSPrefix() + super.getID();
    }

    public String getDDSPrefix() {
        if (isAvoidRosNamespaceConventions()) {
            return "";
        }
        return "rs";
        };
    }
