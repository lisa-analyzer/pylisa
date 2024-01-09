package it.unive.ros.models.pika;

import it.unive.ros.network.NetworkChannel;

public abstract class PikaChannel implements NetworkChannel {
    String ID;

    public PikaChannel(String ID) {
        this.ID = ID;
    }
    @Override
    public String getID() {
        return ID;
    }
}
