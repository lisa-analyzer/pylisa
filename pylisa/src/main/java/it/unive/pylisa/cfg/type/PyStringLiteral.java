package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;


public class PyStringLiteral  extends StringLiteral {

	public PyStringLiteral(CFG cfg, CodeLocation loc, String value) {
		super(cfg, loc, value);
	}
}
