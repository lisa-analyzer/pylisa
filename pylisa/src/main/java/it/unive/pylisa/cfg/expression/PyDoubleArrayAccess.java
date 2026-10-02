package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.DataframeProjection;
import java.util.Set;

public class PyDoubleArrayAccess extends TernaryExpression {

	public PyDoubleArrayAccess(
			CFG cfg,
			CodeLocation loc,
			Type staticType,
			Expression receiver,
			Expression index1,
			Expression index2) {
		super(cfg, loc, "[]", staticType, receiver, index1, index2);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdTernarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression middle,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Type dereferencedType = Untyped.INSTANCE;
		Type firstAccessedType = Untyped.INSTANCE;
		Type childType = Untyped.INSTANCE;

		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfreftype = dftype.getReference();
			Set<Type> rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
			if (rts.stream().anyMatch(t -> t.equals(dfreftype))) {
				HeapDereference deref = new HeapDereference(dftype, left, getLocation());
				it.unive.lisa.symbolic.value.TernaryExpression dfAccess = new it.unive.lisa.symbolic.value.TernaryExpression(
						dftype,
						deref, middle, right,
						new DataframeProjection(0),
						getLocation());
				state = interprocedural.getAnalysis().smallStepSemantics(state, dfAccess, this);
				dereferencedType = dftype;
				firstAccessedType = dftype;
				childType = dfreftype;
			}
		}

		HeapDereference deref = new HeapDereference(dereferencedType, left, getLocation());
		AccessChild firstAccess = new AccessChild(firstAccessedType, deref, middle, getLocation());
		AnalysisState<A> tmp = interprocedural.getAnalysis().smallStepSemantics(state, firstAccess, this);
		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression accessed : tmp.getExecution().getComputedExpressions()) {
			SymbolicExpression cont;
			if (accessed instanceof HeapReference)
				cont = accessed;
			else
				cont = new HeapReference(dereferencedType, accessed, getLocation());

			deref = new HeapDereference(firstAccessedType, cont, getLocation());

			if (childType.isPointerType()) {
				Type inner = childType.asPointerType().getInnerType();
				AccessChild access = new AccessChild(inner, deref, right, getLocation());
				HeapReference ref = new HeapReference(childType, access, getLocation());
				result = result.lub(interprocedural.getAnalysis().smallStepSemantics(tmp, ref, this));
			} else {
				AccessChild access = new AccessChild(childType, deref, right, getLocation());
				result = result.lub(interprocedural.getAnalysis().smallStepSemantics(tmp, access, this));
			}
		}
		return result;
	}
}
