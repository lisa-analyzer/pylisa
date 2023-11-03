package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyComplexType;
import org.apache.commons.lang3.tuple.Pair;

public class PyComplexLiteral extends Literal<Pair<Float, Float>> {

	public PyComplexLiteral(
			CFG cfg,
			CodeLocation location,
			float real,
			float imag) {
		super(cfg, location, Pair.of(real, imag), PyClassType.lookup(PyComplexType.NAME));
	}
}