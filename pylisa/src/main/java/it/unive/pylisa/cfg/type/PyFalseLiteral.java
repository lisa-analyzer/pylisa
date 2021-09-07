package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Literal;
import it.unive.lisa.symbolic.types.BoolType;
import it.unive.lisa.program.cfg.CodeLocation;

public class PyFalseLiteral extends Literal {

	public PyFalseLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc, false, BoolType.INSTANCE);
	}

}
