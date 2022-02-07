package it.unive.pylisa.symbolic;

import it.unive.lisa.symbolic.SymbolicExpression;

public interface DataframeOperatorWithSideEffects {

	SymbolicExpression getDataFrame(SymbolicExpression container);
}
