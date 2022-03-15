package it.unive.pylisa.cfg.expression;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

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
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.Int32;
import it.unive.pylisa.cfg.type.PyTupleType;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.WriteColumn;

public class PyAssign extends Assignment {

	public PyAssign(CFG cfg, CodeLocation location, Expression target, Expression expression) {
		super(cfg, location, target, expression);
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
		if (left instanceof AccessChild && left.getRuntimeTypes().anyMatch(t -> t.equals(PandasSeriesType.REFERENCE))) {
			HeapDereference container_left = (HeapDereference) ((AccessChild) left).getContainer();
			SymbolicExpression r = right;
			if (container_left.getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.INSTANCE))) {
				if (right instanceof AccessChild
						&& right.getRuntimeTypes().anyMatch(t -> t.equals(PandasSeriesType.REFERENCE))) {
					HeapDereference container_right = (HeapDereference) ((AccessChild) right).getContainer();
					if (container_right.getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
						r = container_right;
				}

				return state
						.smallStepSemantics(new BinaryExpression(PandasDataframeType.INSTANCE, container_left, r,
								WriteColumn.INSTANCE, getLocation()), this);
			}
		}

		Expression lefthand = getLeft();
		if (!(lefthand instanceof TupleCreation))
			return super.binarySemantics(interprocedural, state, left, right, expressions);

		// get the variables being assigned
		Expression[] vars = ((TupleCreation) lefthand).getSubExpressions();
		List<ExpressionSet<SymbolicExpression>> ids = Arrays.stream(vars)
				.map(v -> expressions.getState(v).getComputedExpressions()).collect(Collectors.toList());

		// assign the pairs
		AnalysisState<A, H, V, T> assign = state;
		HeapReference ref = new HeapReference(PyTupleType.INSTANCE, right, getLocation());
		HeapDereference deref = new HeapDereference(PyTupleType.INSTANCE, ref, getLocation());

		for (int i = 0; i < ids.size(); i++) {
			ExpressionSet<SymbolicExpression> id = ids.get(i);

			AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref,
					new Constant(Int32.INSTANCE, i, getLocation()),
					getLocation());
			AnalysisState<A, H, V, T> fieldState = assign.smallStepSemantics(fieldAcc, this);

			AnalysisState<A, H, V, T> fieldResult = state.bottom();
			for (SymbolicExpression single : id)
				for (SymbolicExpression lenId : fieldState.getComputedExpressions())
					fieldResult = fieldResult.lub(fieldState.assign(single, lenId, this));
			assign = assign.lub(fieldResult);
		}

		// we leave the reference on the stack
		return assign.smallStepSemantics(ref, this);
	}
}
