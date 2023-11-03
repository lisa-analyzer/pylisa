package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyIntegralType;

public class PyIntLiteral extends Literal<Integer> {

	public PyIntLiteral(
			CFG cfg,
			CodeLocation location,
			int value) {
		super(cfg, location, value, PyClassType.lookup(PyIntegralType.NAME));
	}
}