package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;

public class PyTrueLiteral extends Literal<Boolean> {

	public PyTrueLiteral(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location, true, cfg.getDescriptor().getUnit().getProgram().getTypes().getBooleanType());
	}
}
