package it.unive.pylisa.libraries.standardLibrary;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.NoOpFunction;

public class Print extends NoOpFunction {
	public Print(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	public static Print build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Print(cfg, location, "print", exprs);
	}
}
