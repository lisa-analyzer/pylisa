package it.unive.pylisa.libraries;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;

/**
 * Native implementation of {@code int.__rdivmod__(self, other)} and
 * {@code float.__rdivmod__(self, other)}: {@code divmod(other, self)}. See
 * {@link DivMod}.
 */
public class RDivMod extends DivMod {

	protected RDivMod(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__rdivmod__", true, params);
	}

	public static RDivMod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RDivMod(cfg, location, exprs);
	}
}
