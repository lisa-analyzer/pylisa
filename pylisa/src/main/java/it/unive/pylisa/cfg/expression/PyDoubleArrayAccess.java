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
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRowsColumns;

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

		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		Type firstLevel = Untyped.INSTANCE;
		Type firstLevelDeref = Untyped.INSTANCE;
		Type secondLevel = Untyped.INSTANCE;
		if (left.getRuntimeTypes().anyMatch(t -> t.equals(dfref))) {
			it.unive.lisa.symbolic.value.TernaryExpression dfAccess = new it.unive.lisa.symbolic.value.TernaryExpression(
					dftype,
					deref, middle, right,
					AccessRowsColumns.INSTANCE,
					getLocation());
			tmp = tmp.smallStepSemantics(dfAccess, this);
			firstLevel = dfref;
			secondLevel = dfref;
			firstLevelDeref = dftype;
		}

		AccessChild access = new AccessChild(firstLevel, deref, middle, getLocation());
		tmp = tmp.smallStepSemantics(access, this);
		AnalysisState<A, H, V, T> result = state.bottom();
		for (SymbolicExpression expr : tmp.getComputedExpressions()) {
			SymbolicExpression cont = expr instanceof HeapReference
					? expr
					: new HeapReference(firstLevel, expr, getLocation());
			deref = new HeapDereference(firstLevelDeref, cont, getLocation());
			access = new AccessChild(secondLevel, deref, right, getLocation());
			result = result.lub(tmp.smallStepSemantics(access, this));
		}
		return result;
	}
}
