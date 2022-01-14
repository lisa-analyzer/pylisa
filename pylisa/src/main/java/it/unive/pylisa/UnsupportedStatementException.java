package it.unive.pylisa;

public class UnsupportedStatementException extends RuntimeException {

	private static final long serialVersionUID = 3217861037317417216L;

	public UnsupportedStatementException() {
		super();
	}

	public UnsupportedStatementException(String s) {
		super(s);
	}

	public UnsupportedStatementException(String s, Exception e) {
		super(s, e);
	}
}
