package it.unive.pylisa.cfg.expression;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.evaluation.RightToLeftEvaluation;
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

public class PyAssign extends Assignment {

	public PyAssign(
			CFG cfg,
			CodeLocation location,
			Expression target,
			Expression expression) {
		super(cfg, location, RightToLeftEvaluation.INSTANCE, target, expression);
	}

	
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression left,
			SymbolicExpression right, StatementStore<A> expressions) throws SemanticException {
		CodeLocation loc = getLocation();

		Analysis<A, D> analysis = interprocedural.getAnalysis();
		
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
			Type dfreftype = dftype.getReference();
			PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
			Type seriesreftype = seriestype.getReference();

			if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(left, this, analysis, state)) {
				HeapDereference lderef = PandasSemantics.getDataframeDereference(left, this, analysis, state);
				SymbolicExpression write;
				Set<Type> rts = analysis.getRuntimeTypesOf(state, right, this);
				if (rts.stream().anyMatch(t -> t.equals(dfreftype) || t.equals(seriesreftype))) {
					// asssigning part of a dataframe to another dataframe so
					// get deref from right
					if (PandasSemantics.isDataframePortionThatCanBeAssignedTo(right, this, analysis, state))
						right = PandasSemantics.getDataframeDereference(right, this, analysis, state);
					write = new BinaryExpression(dftype, lderef, right, new AssignToSelection(0), loc);
				} else
					// assigning a part of a dataframe to a constant
					write = new BinaryExpression(dftype, lderef, right, new AssignToConstant(0), loc);

				// we leave on the stack the column that received the assignment
				return analysis.smallStepSemantics(analysis.smallStepSemantics(state, write, this), left, this);
			}
		}

		Expression lefthand = getLeft();
		if (!(lefthand instanceof TupleCreation))
			return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);

		// get the variables being assigned
		Expression[] vars = ((TupleCreation) lefthand).getSubExpressions();
		List<ExpressionSet> ids = Arrays.stream(vars)
				.map(v -> expressions.getState(v).getExecutionExpressions()).collect(Collectors.toList());

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
			AnalysisState<A> fieldState = analysis.smallStepSemantics(assign, fieldAcc, this);

			AnalysisState<A> fieldResult = state.bottom();
			for (SymbolicExpression single : id)
				for (SymbolicExpression lenId : fieldState.getExecutionExpressions())
					fieldResult = fieldResult.lub(analysis.assign(fieldState, single, lenId, this));
			assign = assign.lub(fieldResult);
		}

		// we leave the reference on the stack
		return interprocedural.getAnalysis().smallStepSemantics(assign, ref, this);
	}
}