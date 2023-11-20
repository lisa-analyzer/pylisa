package it.unive.ros.models.rclpy;

public class Topic {
    private String name;
    public Topic(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }
}
