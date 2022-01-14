package it.unive.pylisa.analysis.libraries.warnings;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.analysis.libraries.NoEffectMethod;

public class FilterWarnings extends NoEffectMethod {
	public FilterWarnings(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	public static FilterWarnings build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new FilterWarnings(cfg, location, "filterWarnings", exprs);
	}
}
