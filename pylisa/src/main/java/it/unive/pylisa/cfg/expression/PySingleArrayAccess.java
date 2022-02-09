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
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ColumnAccess;

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
		Type dereferencedType = null;
		Type childType = getStaticType();
		for (Type t : left.getRuntimeTypes())
			if (t.isPointerType()) {
				ExternalSet<Type> inner = t.asPointerType().getInnerTypes();

				Type tmp = inner.isEmpty() ? Untyped.INSTANCE
						: inner.reduce(inner.first(), (r, tt) -> r.commonSupertype(tt));
				if (dereferencedType == null)
					dereferencedType = tmp;
				else
					dereferencedType = dereferencedType.commonSupertype(tmp);
			}
		if (dereferencedType == null)
			dereferencedType = Untyped.INSTANCE;

		if (left.getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.REFERENCE))) {
			it.unive.lisa.symbolic.value.BinaryExpression col = new it.unive.lisa.symbolic.value.BinaryExpression(
					PandasSeriesType.INSTANCE, left, right, ColumnAccess.INSTANCE, getLocation());
			result = result.smallStepSemantics(col, this);
			childType = PandasSeriesType.REFERENCE;
		}

		HeapDereference deref = new HeapDereference(dereferencedType, left, getLocation());
		AccessChild access = new AccessChild(childType, deref, right, getLocation());
		return result.smallStepSemantics(access, this);
	}
}
