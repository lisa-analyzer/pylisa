package it.unive.pylisa.symbolic;

import it.unive.lisa.symbolic.SymbolicExpression;

public interface SideEffectOperator {

	SymbolicExpression getDataFrame(SymbolicExpression container);
}
