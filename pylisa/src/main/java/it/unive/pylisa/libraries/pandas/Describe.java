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
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.Statistics;

public class Describe extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	public Describe(CFG cfg, CodeLocation location, String constructName, Expression dataframe) {
		super(cfg, location, constructName, dataframe);
	}

	public static Describe build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Describe(cfg, location, "describe", exprs[0]);
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
		HeapDereference deref = new HeapDereference(PandasDataframeType.INSTANCE, expr, getLocation());
		UnaryExpression desc = new UnaryExpression(PandasDataframeType.INSTANCE, deref, Statistics.INSTANCE,
				getLocation());
		AnalysisState<A, H, V, T> dState = state.smallStepSemantics(desc, st);
		return dState.smallStepSemantics(expr, st);
		// TODO head effectively creates a new dataframe, we should implement
				// that as a semantic
	}
}
