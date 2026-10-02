package it.unive.pylisa.cfg.statement;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyModuleType;

public class ModuleLiteral extends Literal<CompilationUnit> {

	/**
	 * Builds a typed literal, consisting of a constant value, happening at the
	 * given location in the program.
	 *
	 * @param cfg      the cfg that this expression belongs to
	 * @param location the location where the expression is defined within the
	 *                     program
	 * @param value    the value of this literal
	 */
	public ModuleLiteral(
			CFG cfg,
			CodeLocation location,
			CompilationUnit value) {
		super(cfg, location, value, PyModuleType.lookup(value.getName()));
	}

	@Override
	public String toString() {
		return "<module> " + getValue().toString();
	}
}
