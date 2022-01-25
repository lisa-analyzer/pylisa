package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;

public class PyDoubleArrayAccess extends TernaryExpression {

	public PyDoubleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index1,
			Expression index2) {
		super(cfg, loc, "[]", staticType, receiver, index1, index2);
	}

	@Override
	protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H,
			V> ternarySemantics(InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> state,
					SymbolicExpression left, SymbolicExpression middle, SymbolicExpression right,
					StatementStore<A, H, V> expressions)
					throws SemanticException {
		throw new SemanticException("Not yer supported");
	}
}
