package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.expression.literal.PyMissingLiteral;

/**
 * The {@code missing} default of a parameter: the parameter was not passed
 * (see {@link PyMissingLiteral}).
 */
public class MissingValue implements Value {

	@Override
	public String toString() {
		return "MissingValue";
	}

	@Override
	public Expression toLiSAExpression(
			CFG init) {
		return new PyMissingLiteral(init, init.getDescriptor().getLocation());
	}
}
