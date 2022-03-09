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
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ApplyTransformation;

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
		CodeLocation location = getLocation();
		HeapDereference deref = new HeapDereference(PandasSeriesType.INSTANCE, expr, getLocation());
		AnalysisState<A, H, V, T> result = state.bottom();

		// we allocate the copy of the receiver that will contain the converted
		// portion
		HeapAllocation allocation = new HeapAllocation(PandasSeriesType.INSTANCE, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, st);
		AnalysisState<A, H, V, T> copy = state.bottom();
		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			// copy the dataframe
			AnalysisState<A, H, V, T> assigned = allocated.assign(loc, deref, st);
			for (SymbolicExpression id : assigned.getComputedExpressions()) {
				// the new dataframe will receive the conversion
				UnaryExpression transform = new UnaryExpression(PandasSeriesType.INSTANCE, id,
						new ApplyTransformation(ApplyTransformation.Kind.TO_DATETIME),
						location);
				copy = copy.lub(assigned.smallStepSemantics(transform, st));
			}

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(PandasDataframeType.REFERENCE, loc, location);
			result = result.lub(copy.smallStepSemantics(ref, st));
		}

		return result;
	}
}
