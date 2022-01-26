package it.unive.pylisa.cfg.statement;

import java.util.Arrays;
import java.util.List;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.caches.Caches;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
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
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.type.PyTupleType;

public class TupleCreation extends Expression {
	private final List<Expression> values;

	public TupleCreation(List<Expression> values, CFG cfg, CodeLocation loc) {
		super(cfg, loc);

		this.values = values;
	}

	@Override
	public int setOffset(int i) {
		super.offset = i;
		return i;
	}

	@Override
	public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
		return false;
	}

	@Override
	public String toString() {
		return "(" + Arrays.toString(values.toArray()) + ")";
	}

	public List<Expression> getValues() {
		return values;
	}
	
	@Override
	public <A extends AbstractState<A, H, V>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>> AnalysisState<A, H, V> semantics(AnalysisState<A, H, V> state,
					InterproceduralAnalysis<A, H, V> interprocedural, StatementStore<A, H, V> expressions)
					throws SemanticException {
		AnalysisState<A, H, V> result = state.bottom();
		ExternalSet<Type> type = Caches.types().mkSingletonSet(PyTupleType.INSTANCE);
		ExternalSet<Type> untyped = Caches.types().mkSingletonSet(Untyped.INSTANCE);

		// allocate the heap region
		HeapAllocation alloc = new HeapAllocation(type, getLocation());
		AnalysisState<A, H, V> sem = state.smallStepSemantics(alloc, this);

		// assign the pairs
		AnalysisState<A, H, V> assign = sem;
		for (SymbolicExpression loc : sem.getComputedExpressions()) {
			HeapReference ref = new HeapReference(type, loc, getLocation());
			HeapDereference deref = new HeapDereference(type, ref, getLocation());

			for (int i = 0; i < values.size(); i++) {
				Expression pair = values.get(i);
				AnalysisState<A, H, V> value = pair.semantics(assign, interprocedural, expressions);
				
				AnalysisState<A, H, V> fieldResult = state.bottom();
				AccessChild fieldAcc = new AccessChild(untyped, deref, new Constant(Int32.INSTANCE, i, getLocation()), getLocation());
				for (SymbolicExpression init : value.getComputedExpressions()) {
					AnalysisState<A, H, V> fieldState = value.smallStepSemantics(fieldAcc, this);
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
