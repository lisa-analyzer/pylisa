package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;

public class DataFrameConstant extends Constant {

	/**
	 * Builds the constant.
	 *
	 * @param type     the type of the constant
	 * @param value    the constant value
	 * @param location the code location of the statement that has generated
	 */
	public DataFrameConstant(Type type, SymbolicExpression value, CodeLocation location) {
		super(type, value, location);
	}

	@Override
	public SymbolicExpression getValue() {
		return (SymbolicExpression) super.getValue();
	}
}
