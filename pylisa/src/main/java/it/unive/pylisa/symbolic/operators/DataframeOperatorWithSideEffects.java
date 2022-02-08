package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.SymbolicExpression;

public interface DataframeOperatorWithSideEffects {

	SymbolicExpression getDataFrame(SymbolicExpression container);
}
