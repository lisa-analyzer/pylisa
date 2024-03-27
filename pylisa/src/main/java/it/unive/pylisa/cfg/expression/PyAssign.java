package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.dataframes.AssignToConstant;
import it.unive.pylisa.symbolic.operators.dataframes.AssignToSelection;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class PyAssign extends Assignment {

	public PyAssign(
			CFG cfg,
			CodeLocation location,
			Expression target,
			Expression expression) {
		super(cfg, location, target, expression);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();

		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfreftype = dftype.getReference();
			PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
			Type seriesreftype = seriestype.getReference();

			if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(left, this, state.getState())) {
				HeapDereference lderef = PandasSemantics.getDataframeDereference(left, this, state.getState());
				SymbolicExpression write;
				Set<Type> rts = state.getState().getRuntimeTypesOf(right, this, state.getState());
				if (rts.stream().anyMatch(t -> t.equals(dfreftype) || t.equals(seriesreftype))) {
					// asssigning part of a dataframe to another dataframe so
					// get deref from right
					if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(right, this, state.getState()))
						right = PandasSemantics.getDataframeDereference(right, this, state.getState());
					write = new BinaryExpression(dftype, lderef, right, new AssignToSelection(0), loc);
				} else
					// assigning a part of a dataframe to a constant
					write = new BinaryExpression(dftype, lderef, right, new AssignToConstant(0), loc);

				// we leave on the stack the column that received the assignment
				return state.smallStepSemantics(write, this).smallStepSemantics(left, this);
			}
		}

		Expression lefthand = getLeft();
		if (!(lefthand instanceof TupleCreation))
			return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);

		// get the variables being assigned
		Expression[] vars = ((TupleCreation) lefthand).getSubExpressions();
		List<ExpressionSet> ids = Arrays.stream(vars)
				.map(v -> expressions.getState(v).getComputedExpressions()).collect(Collectors.toList());

		// assign the pairs
		AnalysisState<A> assign = state;

		Type type = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);
		HeapReference ref = new HeapReference(type, right, loc);
		HeapDereference deref = new HeapDereference(type, ref, loc);

		for (int i = 0; i < ids.size(); i++) {
			ExpressionSet id = ids.get(i);

			AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref,
					new Constant(Int32Type.INSTANCE, i, loc),
					loc);
			AnalysisState<A> fieldState = assign.smallStepSemantics(fieldAcc, this);

			AnalysisState<A> fieldResult = state.bottom();
			for (SymbolicExpression single : id)
				for (SymbolicExpression lenId : fieldState.getComputedExpressions())
					fieldResult = fieldResult.lub(fieldState.assign(single, lenId, this));
			assign = assign.lub(fieldResult);
		}

		// we leave the reference on the stack
		return assign.smallStepSemantics(ref, this);
	}
}
