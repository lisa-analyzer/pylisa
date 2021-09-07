package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Literal;
import it.unive.lisa.symbolic.types.BoolType;

public class PyNoneLiteral extends Literal {

	public PyNoneLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc, false, BoolType.INSTANCE);
	}

}
