package it.unive.pylisa.cfg.expression.binary;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.program.cfg.statement.call.BinaryNativeCall;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.type.PyBoolType;

public class PyNot2 extends BinaryNativeCall {

	public PyNot2(CFG cfg, CodeLocation loc, Expression left, Expression right) {
		super(cfg, loc, "!=", PyBoolType.INSTANCE, left, right);
	}


	@Override
	protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> binarySemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> leftState, SymbolicExpression leftExp, AnalysisState<A, H, V> rightState, SymbolicExpression rightExp) {
		throw new UnsupportedStatementException();
	}
}
