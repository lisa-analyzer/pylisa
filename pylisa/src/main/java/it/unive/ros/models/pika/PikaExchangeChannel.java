package it.unive.ros.models.pika;

import java.util.ArrayList;
import java.util.List;

public class PikaExchangeChannel extends PikaChannel{

    private List<PikaQueueBinding> queueBindings = new ArrayList<>();

    private String exchangeType;
    public PikaExchangeChannel(String ID, String exchangeType) throws Exception {
        super(ID);
        if (!exchangeType.equals("direct") && !exchangeType.equals("fanout") && !exchangeType.equals("topic") && !exchangeType.equals("headers")) {
            throw new Exception("Invalid exchangeType!");
        }
        this.exchangeType = exchangeType;
    }
    public String getExchangeType() {
        return exchangeType;
    }
    public void bindQueue(PikaQueueChannel queue, String routingKey) {
        for (PikaQueueBinding binding : queueBindings) {
            if (binding.getQueue().getID().equals(queue.getID()) && binding.getRoutingKey().equals(routingKey)) {
                return;
            }
        }
        queueBindings.add(new PikaQueueBinding(queue, this, routingKey));
    }

    public List<PikaQueueBinding> getQueueBindings() {
        return queueBindings;
    }

    public List<PikaQueueBinding> getQueueBindings(String routingKey) {
        List<PikaQueueBinding> result = new ArrayList<>();
        for (PikaQueueBinding b : queueBindings) {
            if (b.getRoutingKey().equals(routingKey)) {
                result.add(b);
            }
        }
        return result;
    }
}
