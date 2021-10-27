package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;

public class PyTrueLiteral extends TrueLiteral {

	public PyTrueLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc);
	}
}
