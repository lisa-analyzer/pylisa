package it.unive.ros.network;

import it.unive.ros.models.rclpy.ROSCommunicationChannel;
import it.unive.ros.models.rclpy.ROSNetworkEntity;

import java.util.ArrayList;
import java.util.List;

public interface NetworkEntityContainer<Entity extends NetworkEntity> {
    String getID();

    String getName();

    String getURI();
    List<Entity> getNetworkEntities();

    List<NetworkEvent> getProcessedEvents();
    void addNetworkEntity(Entity ne);

}
