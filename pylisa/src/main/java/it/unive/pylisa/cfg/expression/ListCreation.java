package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.caches.Caches;
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
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyListType;

public class ListCreation extends NaryExpression {

	public ListCreation(CFG cfg, CodeLocation loc, Expression... values) {
		super(cfg, loc, "list", values);
	}

	@Override
	public <A extends AbstractState<A, H, V>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>> AnalysisState<A, H, V> expressionSemantics(
					InterproceduralAnalysis<A, H, V> interprocedural,
					AnalysisState<A, H, V> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V> expressions)
					throws SemanticException {
		AnalysisState<A, H, V> result = state.bottom();
		ExternalSet<Type> type = Caches.types().mkSingletonSet(PyListType.INSTANCE);
		ExternalSet<Type> untyped = Caches.types().mkSingletonSet(Untyped.INSTANCE);

		// allocate the heap region
		HeapAllocation alloc = new HeapAllocation(type, getLocation());
		AnalysisState<A, H, V> sem = state.smallStepSemantics(alloc, this);

		// assign the pairs
		AnalysisState<A, H, V> assign = state.bottom();
		for (SymbolicExpression loc : sem.getComputedExpressions()) {
			HeapReference ref = new HeapReference(type, loc, getLocation());
			HeapDereference deref = new HeapDereference(type, ref, getLocation());

			for (int i = 0; i < params.length; i++) {
				AnalysisState<A, H, V> fieldResult = state.bottom();
				Constant idx = new Constant(Int32.INSTANCE, i, getLocation());
				AccessChild fieldAcc = new AccessChild(untyped, deref, idx, getLocation());
				for (SymbolicExpression init : sem.getComputedExpressions()) {
					AnalysisState<A, H, V> fieldState = state.smallStepSemantics(fieldAcc, this);
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
