package it.unive.pylisa.cfg.expression;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;

public class PyStringLiteral extends StringLiteral {

	private String delimiter;

	public PyStringLiteral(CFG cfg, CodeLocation location, String value, String delimiter) {
		super(cfg, location, value);
		this.delimiter = delimiter;
	}

	@Override
	public String toString() {
		return delimiter + getValue() + delimiter;
	}

	public String toString(boolean delimiter) {
		return delimiter ? toString() : getValue();
	}
}
