package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Literal;
import it.unive.lisa.symbolic.types.BoolType;

public class PyTrueLiteral extends Literal {

	public PyTrueLiteral(CFG cfg, CodeLocation loc) {
		super(cfg, loc, true, BoolType.INSTANCE);
	}
}
