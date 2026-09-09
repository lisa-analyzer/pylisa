package it.unive.ros.models.rclpy;

import it.unive.ros.network.Network;
import it.unive.ros.network.NetworkChannel;
import it.unive.ros.network.NetworkEvent;
import it.unive.ros.network.NetworkMessage;
import java.util.List;

public class PublishMessageEvent implements NetworkEvent {
	ROSNetworkEntity initiator;
	NetworkEvent parent;

	NetworkMessage message;

	public PublishMessageEvent(
			NetworkMessage message) {
		this.message = message;
	}

	@Override
	public ROSNetworkEntity getInitiator() {
		return initiator;
	}

	@Override
	public void setInitiator(
			ROSNetworkEntity ne) {
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
	public void process(
			Network network)
			throws Exception {
		String channelId = this.getChannel().getID();
		List<ROSNetworkEntity<?>> nes = network.getChannelNetworkEntities(channelId);
		for (ROSNetworkEntity<?> ne : nes) {
			if (ne instanceof ROSTopicSubscription) {
				if (ne.getContainer() != null) {
					// skip dangling entities
					ne.processMessage(this.getMessage());
				}

			}
		}
	}
}
