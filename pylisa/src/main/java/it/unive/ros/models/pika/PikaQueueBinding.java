package it.unive.ros.models.pika;

public class PikaQueueBinding {
    private PikaQueueChannel queue;
    private PikaExchangeChannel exchange;

    public PikaQueueChannel getQueue() {
        return queue;
    }

    public PikaExchangeChannel getExchange() {
        return exchange;
    }

    public String getRoutingKey() {
        return routingKey;
    }

    private String routingKey;

    public PikaQueueBinding(PikaQueueChannel queue, PikaExchangeChannel exchange, String routingKey) {
        this.queue = queue;
        this.exchange = exchange;
        this.routingKey = routingKey;
    }

}
