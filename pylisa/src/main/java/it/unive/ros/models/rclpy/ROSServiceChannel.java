package it.unive.ros.models.rclpy;

public class ROSServiceChannel extends ROSCommunicationChannel{
    public ROSServiceChannel(String ID) {
        super(ID);
    }


    public ROSServiceChannel(String id, boolean system) {
        super(id, system);
    }
}
