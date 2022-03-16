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
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.PopSelection;

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
		CodeLocation loc = getLocation();
		AnalysisState<A, H, V, T> result = state.bottom();

		SymbolicExpression df = ((AccessChild) expr).getContainer();
		// we allocate the copy that will contain the converted portion
		AnalysisState<A, H, V, T> copied = PandasSemantics.copyDataframe(state, df, st);

		ApplyTransformation op = new ApplyTransformation(ApplyTransformation.Kind.TO_DATETIME);
		UnaryExpression pop = new UnaryExpression(PandasDataframeType.INSTANCE, df, PopSelection.INSTANCE, loc);
		for (SymbolicExpression id : copied.getComputedExpressions()) {
			// the new dataframe will receive the conversion
			UnaryExpression transform = new UnaryExpression(PandasSeriesType.INSTANCE, id, op, loc);
			AnalysisState<A, H, V, T> tmp = copied.smallStepSemantics(transform, st);

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(PandasDataframeType.REFERENCE, id, loc);
			result = result.lub(tmp.smallStepSemantics(pop, st).smallStepSemantics(ref, st));
		}

		return result;
	}
}
