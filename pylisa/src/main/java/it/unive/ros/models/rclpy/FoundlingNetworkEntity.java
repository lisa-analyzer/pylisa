package it.unive.ros.models.rclpy;

import it.unive.ros.network.NetworkEntity;
import it.unive.ros.network.NetworkEntityContainer;

public interface FoundlingNetworkEntity {

    NetworkEntity toNetworkEntity(NetworkEntityContainer n);
    String getParentName();

}
