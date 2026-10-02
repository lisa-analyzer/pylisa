package it.unive.pylisa;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;

public class UnsupportedStatementException extends RuntimeException {

	private static final long serialVersionUID = 3217861037317417216L;

	public UnsupportedStatementException() {
		super();
	}

	public UnsupportedStatementException(
			String message) {
		super(message);
	}

	public UnsupportedStatementException(
			String message,
			CodeLocation location) {
		super(message + " at " + location);
	}

	public UnsupportedStatementException(
			Statement s) {
		super("Not supported yet:" + s);
	}
}
