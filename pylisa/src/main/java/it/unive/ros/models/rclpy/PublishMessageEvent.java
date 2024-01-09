package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.SemanticException;
import it.unive.ros.network.*;

import java.util.List;

public class PublishMessageEvent implements NetworkEvent {
    NetworkEntity initiator;
    NetworkEvent parent;

    NetworkMessage message;

    public PublishMessageEvent(NetworkMessage message) {
        this.message = message;
    }
    @Override
    public NetworkEntity getInitiator() {
        return initiator;
    }

    @Override
    public void setInitiator(NetworkEntity ne) {
        this.initiator = ne;
    }

    @Override
    public NetworkChannel getChannel() {
        return initiator.getChannel();
    }

    @Override
    public NetworkMessage getMessage() {
        return message;
    }

    @Override
    public NetworkEvent getParent() {
        return this.parent;
    }

    @Override
    public void process(Network network) throws SemanticException {
        String channelId = this.getChannel().getID();
        List<NetworkEntity> nes = network.getChannelNetworkEntities(channelId);
        for (NetworkEntity ne : nes) {
            if (ne instanceof Subscription) {
                ne.processMessage(this.getMessage());
            }
        }
    }
}
