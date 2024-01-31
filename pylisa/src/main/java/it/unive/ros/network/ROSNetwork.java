package it.unive.ros.network;

import it.unive.ros.models.rclpy.ROSCommunicationChannel;
import it.unive.ros.models.rclpy.ROSNetworkEntity;
import it.unive.ros.models.rclpy.ROSNode;

public class ROSNetwork extends Network<ROSNode, ROSNetworkEntity<? extends ROSCommunicationChannel>, ROSCommunicationChannel>{
}
