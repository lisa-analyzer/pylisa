package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.program.FunctionUnit;

public class FunctionLiteral extends Literal<FunctionUnit> {
	public FunctionLiteral(
			CFG cfg,
			CodeLocation location,
			FunctionUnit unit) {
		super(cfg, location, unit, PyFunctionType.lookup(unit.getName()));
	}

	@Override
	public String toString() {
		return "<function> " + getValue().toString();
	}

	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		FunctionLiteral other = (FunctionLiteral) obj;
		return getValue().equals(other.getValue());
	}

}
