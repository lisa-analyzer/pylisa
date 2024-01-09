package it.unive.ros.network;

public class NetworkMessage {
    NetworkEvent event;
    Object message;

    public NetworkMessage(Object message) {
        this.message = message;
    }

    public void setNetworkEvent(NetworkEvent event) {
        this.event = event;
    }

    public NetworkEvent getNetworkEvent() {
        return event;
    }

    public Object getMessage() { return message; }

}
