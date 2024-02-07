package it.unive.ros.network;

import it.unive.ros.models.rclpy.ROSNetworkEntity;

public interface NetworkEvent {
    ROSNetworkEntity getInitiator();

    void setInitiator(ROSNetworkEntity ne);
    NetworkChannel getChannel();
    NetworkMessage getMessage();

    NetworkEvent getParent();
    void process(Network network) throws Exception;
}
