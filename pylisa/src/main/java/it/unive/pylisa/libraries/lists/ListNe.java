package it.unive.pylisa.libraries.lists;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;

/**
 * Native implementation of {@code list.__ne__(self, other)}, the negation of
 * {@link ListEq}.
 */
public class ListNe extends ListEq {

	protected ListNe(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, true, params);
	}

	public static ListNe build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListNe(cfg, location, exprs);
	}
}
