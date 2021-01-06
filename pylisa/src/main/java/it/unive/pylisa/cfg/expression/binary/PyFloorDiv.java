package it.unive.pylisa.cfg.expression.binary;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.HeapDomain;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.ValueDomain;
import it.unive.lisa.callgraph.CallGraph;
import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.BinaryNativeCall;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;

public class PyFloorDiv extends BinaryNativeCall{

	public PyFloorDiv(CFG cfg, String sourceFile, int line, int col, Expression left, Expression right) {
		super(cfg, sourceFile, line, col, "//", left, right);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected <H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<H, V> binarySemantics(
			AnalysisState<H, V> computedState, CallGraph callGraph, SymbolicExpression left, SymbolicExpression right)
			throws SemanticException {
		// TODO Auto-generated method stub
		return null;
	}

}
