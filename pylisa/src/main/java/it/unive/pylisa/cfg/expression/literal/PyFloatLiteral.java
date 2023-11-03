package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyRealType;

public class PyFloatLiteral extends Literal<Float> {

	public PyFloatLiteral(
			CFG cfg,
			CodeLocation location,
			float value) {
		super(cfg, location, value, PyClassType.lookup(PyRealType.NAME));
	}
}