package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		Type dereferencedType = null;
		Type childType = getStaticType();
		Set<Type> rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
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

			rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
			if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(t -> t.equals(dfref))) {
				it.unive.lisa.symbolic.value.BinaryExpression col = new it.unive.lisa.symbolic.value.BinaryExpression(
						seriestype, deref, right, new ColumnProjection(0), getLocation());
				result = interprocedural.getAnalysis().smallStepSemantics(state, col, this);
				rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, right, this);
				childType = rts.stream().anyMatch(dfref::equals) ? dfref : seriesref;
			}
		}

		if (childType.isPointerType()) {
			Type inner = childType.asPointerType().getInnerType();
			AccessChild access = new AccessChild(inner, deref, right, getLocation());
			HeapReference ref = new HeapReference(childType, access, getLocation());
			return interprocedural.getAnalysis().smallStepSemantics(result, ref, this);
		} else {
			AccessChild access = new AccessChild(childType, deref, right, getLocation());
			return state.lub(interprocedural.getAnalysis().smallStepSemantics(result, access, this));
		}
	}
}
