package it.unive.pylisa.cfg.expression;

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
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.AccessRowsColumns;

public class PyDoubleArrayAccess extends TernaryExpression {

	public PyDoubleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index1,
			Expression index2) {
		super(cfg, loc, "[]", staticType, receiver, index1, index2);
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> ternarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression middle,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		HeapDereference deref = new HeapDereference(getStaticType(), left, getLocation());
		AnalysisState<A, H, V, T> tmp = state;

		if (left.getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.REFERENCE))) {
			it.unive.lisa.symbolic.value.TernaryExpression dfAccess = 
				new it.unive.lisa.symbolic.value.TernaryExpression(
					PandasDataframeType.INSTANCE, 
					deref, middle, right, 
					AccessRowsColumns.INSTANCE, 
					getLocation()
			);
			tmp = tmp.smallStepSemantics(dfAccess, this);
		}

		AccessChild access = new AccessChild(Untyped.INSTANCE, deref, middle, getLocation());
		tmp = tmp.smallStepSemantics(access, this);
		AnalysisState<A, H, V, T> result = state.bottom();
		for (SymbolicExpression expr : tmp.getComputedExpressions()) {
			deref = new HeapDereference(Untyped.INSTANCE, expr, getLocation());
			access = new AccessChild(Untyped.INSTANCE, deref, right, getLocation());
			result = result.lub(tmp.smallStepSemantics(access, this));
		}
		return result;
	}
}
