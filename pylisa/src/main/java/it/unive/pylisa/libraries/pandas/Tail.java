package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.NoEffectMethod;

public class Tail extends NoEffectMethod {
	public Tail(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	public static Tail build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Tail(cfg, location, "tail", exprs);
	}
}
