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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.PyDataframeType;
import it.unive.pylisa.libraries.pandas.PySeriesType;
import it.unive.pylisa.symbolic.ColumnAccess;

public class PySingleArrayAccess extends BinaryExpression {

	public PySingleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index) {
		super(cfg, loc, "[]", staticType, receiver, index);
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = state;

		if (left.getRuntimeTypes().anyMatch(t -> t.equals(PyDataframeType.INSTANCE))) {
			it.unive.lisa.symbolic.value.BinaryExpression col = new it.unive.lisa.symbolic.value.BinaryExpression(
					PySeriesType.INSTANCE, left, right, ColumnAccess.INSTANCE, getLocation());
			result = result.smallStepSemantics(col, this);
		}

		Type dynamic = null;
		for (Type t : left.getRuntimeTypes())
			if (t.isPointerType()) {
				ExternalSet<Type> inner = t.asPointerType().getInnerTypes();

				Type tmp = inner.isEmpty() ? Untyped.INSTANCE
						: inner.reduce(inner.first(), (r, tt) -> r.commonSupertype(tt));
				if (dynamic == null)
					dynamic = tmp;
				else
					dynamic = dynamic.commonSupertype(tmp);
			}
		if (dynamic == null)
			dynamic = Untyped.INSTANCE;

		HeapDereference deref = new HeapDereference(dynamic, left, getLocation());
		AccessChild access = new AccessChild(getStaticType(), deref, right, getLocation());
		return result.smallStepSemantics(access, this);
	}
}
