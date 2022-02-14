package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.TypeConversion;

public class ToDatetime extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	public ToDatetime(CFG cfg, CodeLocation location, String constructName, Expression series) {
		super(cfg, location, constructName, series);
	}

	public static ToDatetime build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new ToDatetime(cfg, location, "to_datetime", exprs[0]);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression expr,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		HeapDereference deref = new HeapDereference(PandasSeriesType.INSTANCE, expr, getLocation());
		UnaryExpression conv = new UnaryExpression(PandasSeriesType.INSTANCE, expr, new TypeConversion("datetime"),
				getLocation());
		AnalysisState<A, H, V, T> cState = state.smallStepSemantics(conv, st);
		AnalysisState<A, H, V, T> result = cState.bottom();
		for (SymbolicExpression converted : cState.getComputedExpressions())
			result = result.lub(cState.smallStepSemantics(converted, st));
		return cState;
		// TODO todatetiem effectively creates a new dataframe, we
		// should implement that as a semantic
	}
}
