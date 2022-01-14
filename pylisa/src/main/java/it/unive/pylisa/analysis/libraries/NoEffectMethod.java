package it.unive.pylisa.analysis.libraries;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;

public class NoEffectMethod extends NaryExpression implements PluggableStatement {

	public NoEffectMethod(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
		super(cfg, location, constructName, parameter);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
	}

	@Override
	public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H,
			V> expressionSemantics(InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> state,
					ExpressionSet<SymbolicExpression>[] params) throws SemanticException {
		return state;
	}
}