package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.symbolic.PyBytes;

/**
 * A bytes literal ({@code b"..."}).
 */
public class PyBytesLiteral extends Literal<PyBytes> {

	/**
	 * Builds the literal.
	 *
	 * @param cfg      the cfg where the literal is
	 * @param location the location of the literal
	 * @param value    its value
	 */
	public PyBytesLiteral(
			CFG cfg,
			CodeLocation location,
			PyBytes value) {
		super(cfg, location, value, PyBytesType.INSTANCE);
	}
}
