package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.NullLiteral;

public class PyNoneLiteral extends NullLiteral {

	public PyNoneLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc);
	}

}
