package it.unive.ros.network;

import java.util.List;

public interface NetworkEntityContainer<Entity extends NetworkEntity> {
	String getID();

	String getName();

	String getURI();

	List<Entity> getNetworkEntities();

	List<NetworkEvent> getProcessedEvents();

	void addNetworkEntity(
			Entity ne);

}
