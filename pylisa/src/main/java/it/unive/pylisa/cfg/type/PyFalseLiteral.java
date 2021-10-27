package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.type.common.BoolType;

public class PyFalseLiteral extends FalseLiteral {

	public PyFalseLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc);
	}

}
