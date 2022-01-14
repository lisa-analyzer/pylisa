package it.unive.pylisa.analysis.libraries.pandas;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.analysis.libraries.NoEffectMethod;

public class Head extends NoEffectMethod {
	public Head(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	public static Head build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Head(cfg, location, "head", exprs);
	}
}
