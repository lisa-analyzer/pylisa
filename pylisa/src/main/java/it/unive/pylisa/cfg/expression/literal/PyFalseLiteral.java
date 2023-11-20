package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;

public class PyFalseLiteral extends Literal<Boolean> {

	public PyFalseLiteral(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location, false, cfg.getDescriptor().getUnit().getProgram().getTypes().getBooleanType());
	}

}
