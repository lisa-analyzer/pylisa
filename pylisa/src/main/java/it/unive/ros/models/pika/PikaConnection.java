package it.unive.ros.models.pika;

import it.unive.ros.network.NetworkEntity;
import it.unive.ros.network.NetworkEntityContainer;
import it.unive.ros.network.NetworkEvent;

import java.util.List;

public class PikaConnection implements NetworkEntityContainer {
    String name;
    String ID;

    public PikaConnection(String name, String ID) {
        this.name = name;
        this.ID = ID;
    }
    @Override
    public String getID() {
        return ID;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public List<NetworkEntity> getNetworkEntities() {
        return null;
    }

    @Override
    public List<NetworkEvent> getProcessedEvents() {
        return null;
    }

    @Override
    public void addNetworkEntity(NetworkEntity ne) {

    }
}
