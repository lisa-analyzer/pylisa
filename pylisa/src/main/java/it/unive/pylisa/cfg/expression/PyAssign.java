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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionConstant;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionDataframe;

public class PyAssign extends Assignment {

	public PyAssign(CFG cfg, CodeLocation location, Expression target, Expression expression) {
		super(cfg, location, target, expression);
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

		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		Type seriesref = ((PyClassType) seriestype).getReference();

		CodeLocation loc = getLocation();
		if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(left)) {
			HeapDereference lderef = PandasSemantics.getDataframeDereference(left);
			SymbolicExpression write;
			TypeSystem types = getProgram().getTypes();
			if (right.getRuntimeTypes(types).stream().anyMatch(t -> t.equals(dfref) || t.equals(seriesref))) {
				// asssigning part of a dataframe to another dataframe so get
				// deref from right
				if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(right))
					right = PandasSemantics.getDataframeDereference(right);
				write = new BinaryExpression(dftype, lderef, right, WriteSelectionDataframe.INSTANCE, loc);
			} else
				// assigning a part of a dataframe to a constant
				write = new BinaryExpression(dftype, lderef, right, WriteSelectionConstant.INSTANCE, loc);

			// we leave on the stack the column that received the assignment
			return state.smallStepSemantics(write, this).smallStepSemantics(left, this);
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

		Type type = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);
		HeapReference ref = new HeapReference(type, right, loc);
		HeapDereference deref = new HeapDereference(type, ref, loc);

		for (int i = 0; i < ids.size(); i++) {
			ExpressionSet<SymbolicExpression> id = ids.get(i);

			AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref,
					new Constant(Int32Type.INSTANCE, i, loc),
					loc);
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
