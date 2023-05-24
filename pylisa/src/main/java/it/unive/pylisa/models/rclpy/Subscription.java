package it.unive.pylisa.models.rclpy;


import it.unive.lisa.program.cfg.statement.call.Call;

public class Subscription extends TopicUser {
    private String callbackFunction;

    public Subscription(Node node, Topic topic, String msgType, String callbackFunction) {
        super(node, topic, msgType);
        this.callbackFunction = callbackFunction;
    }

    public String getCallbackFunction() {
        return callbackFunction;
    }
}