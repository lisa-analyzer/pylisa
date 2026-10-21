package it.unive.pylisa.libraries;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;

/**
 * Native implementation of Python's {@code all(iterable)} (see {@link AllAny}).
 */
public class All extends AllAny {

	protected All(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "all", true, params);
	}

	public static All build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new All(cfg, location, exprs);
	}
}
