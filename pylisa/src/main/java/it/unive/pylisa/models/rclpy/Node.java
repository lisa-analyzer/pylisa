package it.unive.pylisa.models.rclpy;

import it.unive.lisa.interprocedural.ScopeId;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Node {

    private Set<Subscription> subscribers;
    private Set<Publisher> publishers;
    private String name;
    private ScopeId scopeId;
    public Node(String name, ScopeId scopeId) {
        this.name = name;
        subscribers = new HashSet<>();
        publishers = new HashSet<>();
        this.scopeId = scopeId;
    }

    public void addNewSubscriber(Topic topic, String msgType, String callBackFunction) {
        subscribers.add(new Subscription(this, topic, msgType, callBackFunction));
    }

    public void addNewPublisher(Topic topic, String msgType) {
        publishers.add(new Publisher(this, topic, msgType));
    }

    public void removeSubscriber(Subscription s) {
        subscribers.remove(s);
    }

    public void removePublisher(Publisher p) {
        publishers.remove(p);
    }

    public Set<Topic> getSubscribersTopics() {
        HashSet<Topic> topics = new HashSet<>();
        for (Subscription s : subscribers) {
            topics.add(s.getTopic());
        }
        return topics;
    }

    public Set<Topic> getPublisherTopics() {
        HashSet<Topic> topics = new HashSet<>();
        for (Publisher s : publishers) {
            topics.add(s.getTopic());
        }
        return topics;
    }

    public Set<Topic> getAllNodeTopics() {
        Set<Topic> topics = getSubscribersTopics();
        topics.addAll(getPublisherTopics());
        return topics;
    }

    public Set<TopicUser> getAllNodeTopicsUsers() {
        Set<TopicUser> topics = new HashSet<>(subscribers);
        topics.addAll(publishers);
        return topics;
    }

    public String getName() {
        return name;
    }


    public ScopeId getScopeId() {
        return scopeId;
    }

    public Set<Subscription> getSubscribers() {
        return subscribers;
    }

    public Set<Publisher> getPublishers() {
        return publishers;
    }
}
