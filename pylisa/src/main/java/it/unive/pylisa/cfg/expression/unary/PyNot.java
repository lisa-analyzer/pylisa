package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.HeapDomain;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.ValueDomain;
import it.unive.lisa.callgraph.CallGraph;
import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.cfg.statement.NativeCall;
import it.unive.lisa.cfg.statement.UnaryNativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.type.PyBoolType;

public class PyNot extends UnaryNativeCall {

	/**
	 * Builds the logical not.
	 * 
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param sourceFile the source file name where this operation is defined
	 * @param line       the line number where this operation is defined
	 * @param col        the column where this operation is defined
	 * @param expression the operand of this operation
	 */
	public PyNot(CFG cfg, String sourceFile, int line, int col, Expression expression) {
		super(cfg, sourceFile, line, col, "not", PyBoolType.INSTANCE, expression);
	}

	@Override
	protected <H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<H, V> unarySemantics(
			AnalysisState<H, V> computedState, CallGraph callGraph, SymbolicExpression expr) throws SemanticException {
		// TODO Auto-generated method stub
		return null;
	}
}
