package it.unive.ros.models.rclpy;


import it.unive.ros.network.NetworkChannel;

public abstract class ROSCommunicationChannel implements NetworkChannel {
    private String ID;
    private Boolean system = false;

    public ROSCommunicationChannel(String ID) {
        this.ID = ID;
    }

    public ROSCommunicationChannel(String ID, Boolean system) {
        this.ID = ID;
        this.system = system;
    }
    @Override
    public String getID() {
        return ID;
    }

    public String getName() {
        return ID;
    }

    public Boolean isSystem() {
        return system;
    }
}
