package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.NoEffectMethod;

public class Info extends NoEffectMethod {
	public Info(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	public static Info build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Info(cfg, location, "info", exprs);
	}
}
