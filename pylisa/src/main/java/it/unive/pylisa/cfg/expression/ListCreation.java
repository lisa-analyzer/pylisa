package it.unive.pylisa.cfg.expression;
import java.util.HashSet;
import java.util.Set;
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
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.Int32;
import it.unive.pylisa.cfg.type.PyListType;

public class ListCreation extends NaryExpression {
	public ListCreation(CFG cfg, CodeLocation loc, Expression... values) {
		super(cfg, loc, "list", values);
		for (Expression e : values)
			if (!(e instanceof Literal))
				throw new UnsupportedOperationException("Only constants supported for now");// x
																							// =
																							// ['a',
																							// 'a',
																							// 'a']
	}
	
	private class ListConstant extends Constant {
		public ListConstant(Type type, ExpressionSet<Constant>[] value, CodeLocation location) {
			super(type, value, location);
		}
	}

	@Override
	@SuppressWarnings("unchecked")
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		ExpressionSet<Constant>[] constants = new ExpressionSet[params.length];
		for (int i = 0; i < constants.length; i++) {
			Set<Constant> s = new HashSet<>();
			params[i].forEach(p -> s.add((Constant) p));
			constants[i] = new ExpressionSet<>(s);
		}
		ListConstant c = new ListConstant(PyListType.INSTANCE, constants, getLocation());
		return state.smallStepSemantics(c, this);
//		
//		AnalysisState<A, H, V, T> result = state.bottom();
//
//		// allocate the heap region
//		HeapAllocation alloc = new HeapAllocation(PyListType.INSTANCE, getLocation());
//		AnalysisState<A, H, V, T> sem = state.smallStepSemantics(alloc, this);
//
//		// assign the pairs
//		AnalysisState<A, H, V, T> assign = state.bottom();
//		for (SymbolicExpression loc : sem.getComputedExpressions()) {
//			HeapReference ref = new HeapReference(PyListType.INSTANCE, loc, getLocation());
//			HeapDereference deref = new HeapDereference(PyListType.INSTANCE, ref, getLocation());
//
//			for (int i = 0; i < params.length; i++) {
//				AnalysisState<A, H, V, T> fieldResult = state.bottom();
//				Constant idx = new Constant(Int32.INSTANCE, i, getLocation());
//				AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref, idx, getLocation());
//				for (SymbolicExpression init : sem.getComputedExpressions()) {
//					AnalysisState<A, H, V, T> fieldState = state.smallStepSemantics(fieldAcc, this);
//					for (SymbolicExpression lenId : fieldState.getComputedExpressions())
//						fieldResult = fieldResult.lub(fieldState.assign(lenId, init, this));
//				}
//				assign = assign.lub(fieldResult);
//			}
//
//			// we leave the reference on the stack
//			result = result.lub(assign.smallStepSemantics(ref, this));
//		}
//
//		return result;
	}
}