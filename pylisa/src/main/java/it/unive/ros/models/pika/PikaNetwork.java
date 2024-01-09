package it.unive.ros.models.pika;

import it.unive.lisa.analysis.SemanticException;
import it.unive.ros.models.rclpy.PublishMessageEvent;
import it.unive.ros.network.Network;
import it.unive.ros.network.NetworkEntity;
import it.unive.ros.network.NetworkEvent;

public class PikaNetwork extends Network {
    @Override
    public void processEvents() throws Exception {
        while (!getNetworkEvents().isEmpty()) {
            NetworkEvent event = getNetworkEvents().get(0);
            if (event.getInitiator() instanceof PikaPublisher) {
                PikaPublisher publisher = (PikaPublisher) event.getInitiator();
                PikaExchangeChannel exchange = (PikaExchangeChannel) publisher.getChannel();
                if (exchange.getExchangeType().equals("direct")) {
                    String publisherRoutingKey = publisher.getRoutingKey();
                     // send message to all the queues that has the same routingKey of the publisher
                    for (PikaQueueBinding binding : exchange.getQueueBindings(publisherRoutingKey)) {
                        for (NetworkEntity e : getNetworkEntities()) {
                            if (e.getChannel().equals(binding.getQueue())) {
                                e.processMessage(event.getMessage());
                            }
                        }
                    }
                }
                else if (exchange.getExchangeType().equals("fanout")) {
                    // send message to all the queues
                    for (PikaQueueBinding binding : exchange.getQueueBindings()) {
                        for (NetworkEntity e : getNetworkEntities()) {
                            if (e.getChannel().equals(binding.getQueue())) {
                                e.processMessage(event.getMessage());
                            }
                        }
                    }

                }
                getNetworkEvents().remove(0);
            }
        }
        super.processEvents();
    }
}
