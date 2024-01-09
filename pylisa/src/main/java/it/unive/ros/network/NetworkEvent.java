package it.unive.ros.network;

import it.unive.lisa.analysis.SemanticException;

public interface NetworkEvent {
    NetworkEntity getInitiator();

    void setInitiator(NetworkEntity ne);
    NetworkChannel getChannel();
    NetworkMessage getMessage();
    NetworkEvent getParent();
    void process(Network network) throws SemanticException;
}
