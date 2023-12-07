package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnProjection;
import java.util.Set;

public class PySingleArrayAccess extends BinaryExpression {

	public PySingleArrayAccess(
			CFG cfg,
			CodeLocation loc,
			Type staticType,
			Expression receiver,
			Expression index) {
		super(cfg, loc, "[]", staticType, receiver, index);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state;
		Type dereferencedType = null;
		Type childType = getStaticType();
		Set<Type> rts = state.getState().getRuntimeTypesOf(left, this, state.getState());
		for (Type t : rts)
			if (t.isPointerType()) {
				Type inner = t.asPointerType().getInnerType();
				if (dereferencedType == null)
					dereferencedType = inner;
				else
					dereferencedType = dereferencedType.commonSupertype(inner);
			}
		if (dereferencedType == null)
			dereferencedType = Untyped.INSTANCE;

		HeapDereference deref = new HeapDereference(dereferencedType, left, getLocation());

		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfref = dftype.getReference();
			PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
			Type seriesref = seriestype.getReference();

			rts = state.getState().getRuntimeTypesOf(left, this, state.getState());
			if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(t -> t.equals(dfref))) {
				it.unive.lisa.symbolic.value.BinaryExpression col = new it.unive.lisa.symbolic.value.BinaryExpression(
						seriestype, deref, right, new ColumnProjection(0), getLocation());
				result = result.smallStepSemantics(col, this);
				rts = state.getState().getRuntimeTypesOf(right, this, state.getState());
				childType = rts.stream().anyMatch(dfref::equals) ? dfref : seriesref;
			}
		}

		if (childType.isPointerType()) {
			Type inner = childType.asPointerType().getInnerType();
			AccessChild access = new AccessChild(inner, deref, right, getLocation());
			HeapReference ref = new HeapReference(childType, access, getLocation());
			return result.smallStepSemantics(ref, this);
		} else {
			AccessChild access = new AccessChild(childType, deref, right, getLocation());
			return result.smallStepSemantics(access, this);
		}
	}
}
