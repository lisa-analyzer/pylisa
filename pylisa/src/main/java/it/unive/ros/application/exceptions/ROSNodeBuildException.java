package it.unive.ros.application.exceptions;

public class ROSNodeBuildException extends ROSBuildException {

    public ROSNodeBuildException(Exception wrapped) {
        super("ROSNodeBuildException", wrapped);
    }
}
