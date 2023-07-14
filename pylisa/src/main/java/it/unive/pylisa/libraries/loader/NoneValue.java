package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;

public class NoneValue implements Value {

	@Override
	public String toString() {
		return "NoneValue";
	}

	@Override
	public Expression toLiSAExpression(CFG init) {
		return new PyNoneLiteral(init, init.getDescriptor().getLocation());
	}
}
