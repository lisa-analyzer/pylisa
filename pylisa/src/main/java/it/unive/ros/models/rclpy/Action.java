package it.unive.ros.models.rclpy;

import it.unive.ros.network.NetworkEntity;

public abstract class Action implements NetworkEntity {
  private String name;
  private String actionType;

  private String callback;

  public Action(
      String name,
      String actionType,
      String callback) {
    this.name = name;
    this.actionType = actionType;
    this.callback = callback;
  }

  public String getName() {
    return name;
  }

  public String actionType() {
    return actionType;
  }

  public String getCallback() {
    return callback;
  }
}
