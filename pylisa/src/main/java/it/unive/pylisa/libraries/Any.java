package it.unive.pylisa.libraries;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;

/**
 * Native implementation of Python's {@code any(iterable)} (see {@link AllAny}).
 */
public class Any extends AllAny {

	protected Any(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "any", false, params);
	}

	public static Any build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Any(cfg, location, exprs);
	}
}
