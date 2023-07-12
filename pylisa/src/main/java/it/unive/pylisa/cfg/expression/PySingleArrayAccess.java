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
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnProjection;

public class PySingleArrayAccess extends BinaryExpression {

	public PySingleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index) {
		super(cfg, loc, "[]", staticType, receiver, index);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
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
		TypeSystem types = getProgram().getTypes();
		for (Type t : left.getRuntimeTypes(types))
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

			if (left.getRuntimeTypes(types).stream().anyMatch(t -> t.equals(dfref))) {
				it.unive.lisa.symbolic.value.BinaryExpression col = new it.unive.lisa.symbolic.value.BinaryExpression(
						seriestype, deref, right, new ColumnProjection(0), getLocation());
				result = result.smallStepSemantics(col, this);
				childType = right.getRuntimeTypes(types).stream().anyMatch(dfref::equals) ? dfref : seriesref;
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
