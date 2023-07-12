package it.unive.pylisa.cfg.expression;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import java.util.Collections;

public class PyTypeLiteral extends Literal<Unit> {

	/**
	 * Builds a typed literal, consisting of a constant value, happening at the
	 * given location in the program.
	 *
	 * @param cfg      the cfg that this expression belongs to
	 * @param location the location where the expression is defined within the
	 *                     program
	 * @param value    the value of this literal
	 */
	public PyTypeLiteral(CFG cfg, CodeLocation location, Unit value) {
		super(cfg, location, value, new PyTypeTokenType(Collections.singleton(PyClassType.lookup(value.getName()))));
	}
}
