package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.Int32;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class TupleCreation extends NaryExpression {

	public TupleCreation(CFG cfg, CodeLocation loc, Expression... values) {
		super(cfg, loc, "tuple", values);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = state.bottom();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);
		
		// allocate the heap region
		HeapAllocation alloc = new HeapAllocation(tupleType, getLocation());
		AnalysisState<A, H, V, T> sem = state.smallStepSemantics(alloc, this);

		// assign the pairs
		AnalysisState<A, H, V, T> assign = state.bottom();
		for (SymbolicExpression loc : sem.getComputedExpressions()) {
			HeapReference ref = new HeapReference(tupleType, loc, getLocation());
			HeapDereference deref = new HeapDereference(tupleType, ref, getLocation());

			for (int i = 0; i < params.length; i++) {
				AnalysisState<A, H, V, T> fieldResult = state.bottom();
				Constant idx = new Constant(Int32.INSTANCE, i, getLocation());
				AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref, idx, getLocation());
				for (SymbolicExpression init : params[i]) {
					AnalysisState<A, H, V, T> fieldState = sem.smallStepSemantics(fieldAcc, this);
					for (SymbolicExpression lenId : fieldState.getComputedExpressions())
						fieldResult = fieldResult.lub(fieldState.assign(lenId, init, this));
				}
				assign = assign.lub(fieldResult);
			}

			// we leave the reference on the stack
			result = result.lub(assign.smallStepSemantics(ref, this));
		}

		return result;
	}

}
