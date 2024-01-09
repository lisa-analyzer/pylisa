package it.unive.ros.network;

import java.util.ArrayList;
import java.util.List;

public interface NetworkEntityContainer /* aka ROSNode */ {
    public String getID();

    public String getName();
    public List<NetworkEntity> getNetworkEntities();

    public List<NetworkEvent> getProcessedEvents();
    public void addNetworkEntity(NetworkEntity ne);

}
