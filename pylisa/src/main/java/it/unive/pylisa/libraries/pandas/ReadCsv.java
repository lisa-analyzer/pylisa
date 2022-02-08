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
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class ReadCsv extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	private Statement st;

	protected ReadCsv(CFG cfg, CodeLocation location, String constructName, Type staticType, Expression arg) {
		super(cfg, location, constructName, staticType, arg);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	public static ReadCsv build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new ReadCsv(cfg, location, "read_csv", PandasDataframeType.INSTANCE, exprs[0]);
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression arg,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation location = getLocation();
		AnalysisState<A, H, V, T> assigned = state.bottom();

		HeapAllocation allocation = new HeapAllocation(PandasDataframeType.INSTANCE, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, st);

		UnaryExpression read = new UnaryExpression(PandasDataframeType.INSTANCE, arg, ReadDataframe.INSTANCE, location);

		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			HeapReference ref = new HeapReference(PandasDataframeType.REFERENCE, loc, location);
			AnalysisState<A, H, V, T> tmp = state.bottom();
			AnalysisState<A, H, V, T> readState = allocated.smallStepSemantics(read, st);
			for (SymbolicExpression df : readState.getComputedExpressions()) 
				tmp = tmp.lub(readState.assign(loc, df, st));
			assigned = tmp.smallStepSemantics(ref, st);
		}

		return assigned;
	}
}