package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Literal;
import it.unive.lisa.program.cfg.CodeLocation;


public class PyStringLiteral  extends Literal {

	public PyStringLiteral(CFG cfg, CodeLocation loc, String value) {
		super(cfg, loc, value, PyStringType.INSTANCE);
	}
}
