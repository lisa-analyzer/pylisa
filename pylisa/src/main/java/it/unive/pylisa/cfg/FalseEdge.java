package it.unive.pylisa.cfg;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;

// TODO delete this
public class FalseEdge extends it.unive.lisa.program.cfg.edge.FalseEdge {

	public FalseEdge(Statement source, Statement destination) {
		super(source, destination);
	}

	@Override
	public <A extends AbstractState<A, H, V>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>> AnalysisState<A, H, V> traverse(
					AnalysisState<A, H, V> sourceState) throws SemanticException {
		ExpressionSet<SymbolicExpression> exprs = sourceState.getComputedExpressions();
		AnalysisState<A, H, V> result = sourceState.bottom();
		for (SymbolicExpression expr : exprs) {
			AnalysisState<A, H, V> tmp = sourceState
					.assume(new UnaryExpression(expr.getTypes(), expr, LogicalNegation.INSTANCE,
							expr.getCodeLocation()),
							getSource());
			result = result.lub(tmp);
		}
		return result;
	}
	
	@Override
	public FalseEdge newInstance(Statement source, Statement destination) {
		return new FalseEdge(source, destination);
	}
}
