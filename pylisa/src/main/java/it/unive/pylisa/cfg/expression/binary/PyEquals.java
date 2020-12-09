package it.unive.pylisa.cfg.expression.binary;


import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.cfg.statement.NativeCall;
import it.unive.pylisa.cfg.type.PyBoolType;

public class PyEquals extends NativeCall{

	/**
	 * Builds the equality test.
	 * 
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param sourceFile the source file name where this operation is defined
	 * @param line       the line number where this operation is defined
	 * @param col        the column where this operation is defined
	 * @param left       the left-hand side of this operation
	 * @param right      the right-hand side of this operation
	 */
	
	public PyEquals(CFG cfg, String sourceFile, int line, int col, Expression left, Expression right) {
		super(cfg, sourceFile, line, col, "==", PyBoolType.INSTANCE, left, right);
	}
	
	@Override
	protected <H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<H, V> callSemantics(
			AnalysisState<H, V> computedState, CallGraph callGraph, SymbolicExpression left, SymbolicExpression right)
			throws SemanticException {
		return computedState
				.smallStepSemantics(new BinaryExpression(Caches.types().mkSingletonSet(BoolType.INSTANCE), left, right,
						BinaryOperator.COMPARISON_EQ));
	}
}

