package it.unive.pylisa.cfg;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;

// TODO delete this
public class TrueEdge extends it.unive.lisa.program.cfg.edge.TrueEdge {

	public TrueEdge(Statement source, Statement destination) {
		super(source, destination);
	}

	@Override
	public <A extends AbstractState<A, H, V>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>> AnalysisState<A, H, V> traverse(
					AnalysisState<A, H, V> sourceState) throws SemanticException {
		ExpressionSet<SymbolicExpression> exprs = sourceState.getComputedExpressions();
		AnalysisState<A, H, V> result = sourceState.bottom();
		for (SymbolicExpression expr : exprs)
			result = result.lub(sourceState.assume(expr, getSource()));
		return result;
	}
	
	@Override
	public TrueEdge newInstance(Statement source, Statement destination) {
		return new TrueEdge(source, destination);
	}
}
