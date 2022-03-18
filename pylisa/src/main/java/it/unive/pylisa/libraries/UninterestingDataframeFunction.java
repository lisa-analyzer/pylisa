package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;

public class UninterestingDataframeFunction extends NaryExpression implements PluggableStatement {

	protected Statement st;

	public UninterestingDataframeFunction(CFG cfg, CodeLocation location, String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	public static UninterestingDataframeFunction build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new UninterestingDataframeFunction(cfg, location, "uninteresting-func", exprs);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V, T> expressions) throws SemanticException {
		AnalysisState<A, H, V, T> result = state.bottom();
		// we just return the same dataframe
		for (SymbolicExpression df : params[0])
			result = result.lub(state.smallStepSemantics(df, st));
		return result;
	}
}