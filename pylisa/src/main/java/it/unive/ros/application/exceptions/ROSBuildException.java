package it.unive.ros.application.exceptions;

public class ROSBuildException extends Exception {
	private final Exception wrapped;

	private final String context;

	protected ROSBuildException(
			String context,
			Exception wrapped) {
		super(context + ": " + wrapped.getMessage());
		this.context = context;
		this.wrapped = wrapped;
	}

	public ROSBuildException(
			Exception wrapped) {
		this("ROSBuilderException", wrapped);
	}

	public Exception getUnderlyingException() {
		return this.wrapped;
	}

	public String getContext() {
		return this.context;
	}

}
