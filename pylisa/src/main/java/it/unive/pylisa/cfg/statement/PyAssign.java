package it.unive.pylisa.cfg.statement;

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
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.Int32;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyTupleType;
import java.util.List;
import java.util.stream.Collectors;

public class PyAssign extends Assignment {

	public PyAssign(CFG cfg, CodeLocation location, Expression target, Expression expression) {
		super(cfg, location, target, expression);
	}

	@Override
	protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H,
			V> binarySemantics(InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> state,
					SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V> expressions)
					throws SemanticException {
		if (!(getLeft() instanceof TupleCreation))
			return super.binarySemantics(interprocedural, state, left, right, expressions);

		// get the variables being assigned
		List<Expression> vars = ((TupleCreation) getLeft()).getValues();
		List<ExpressionSet<SymbolicExpression>> ids = vars.stream()
				.map(v -> expressions.getState(v).getComputedExpressions()).collect(Collectors.toList());

		// assign to each variable the element on the tuple on the right
		ExternalSet<Type> type = Caches.types().mkSingletonSet(PyTupleType.INSTANCE);
		ExternalSet<Type> untyped = Caches.types().mkSingletonSet(Untyped.INSTANCE);

		// allocate the heap region

		// assign the pairs
		AnalysisState<A, H, V> assign = state;
		HeapReference ref = new HeapReference(type, right, getLocation());
		HeapDereference deref = new HeapDereference(type, ref, getLocation());

		for (int i = 0; i < ids.size(); i++) {
			ExpressionSet<SymbolicExpression> id = ids.get(i);

			AccessChild fieldAcc = new AccessChild(untyped, deref, new Constant(Int32.INSTANCE, i, getLocation()),
					getLocation());
			AnalysisState<A, H, V> fieldState = assign.smallStepSemantics(fieldAcc, this);

			AnalysisState<A, H, V> fieldResult = state.bottom();
			for (SymbolicExpression single : id)
				for (SymbolicExpression lenId : fieldState.getComputedExpressions())
					fieldResult = fieldResult.lub(fieldState.assign(single, lenId, this));
			assign = assign.lub(fieldResult);
		}

		// we leave the reference on the stack
		return assign.smallStepSemantics(ref, this);
	}
}
