package it.unive.testing;

class ParsingRes {
	String fileName;
	String status;
	String error;
	String stackTrace;

	public ParsingRes(String fileName, String status, String error, String stackTrace) {
		this.fileName = fileName;
		this.status = status;
		this.error = error;
		this.stackTrace = stackTrace;
	}
}