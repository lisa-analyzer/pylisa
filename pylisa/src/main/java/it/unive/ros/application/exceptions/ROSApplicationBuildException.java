package it.unive.ros.application.exceptions;

public class ROSApplicationBuildException extends ROSBuildException {
	public ROSApplicationBuildException(Exception wrapped) {
		super("ROSApplicationBuildException", wrapped);
	}
}
