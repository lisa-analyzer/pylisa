package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;

/**
 * Python's {@code left and right} (see {@link PyShortCircuit}).
 */
public class PyAnd extends PyShortCircuit {

	/**
	 * Builds the operator.
	 *
	 * @param cfg      the cfg where the operator is
	 * @param location the location of the operator
	 * @param left     the left operand
	 * @param right    the right operand
	 */
	public PyAnd(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, "&&", left, right);
	}

	@Override
	protected boolean decidedByTrue() {
		return false;
	}
}
