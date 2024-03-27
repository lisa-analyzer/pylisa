package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.CopyDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.SeriesComparison;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;
import java.util.Set;

public class PandasSemantics {

	private PandasSemantics() {
	}

	public static <A extends AbstractState<A>> AnalysisState<A> createAndInitDataframe(
			AnalysisState<A> state,
			SymbolicExpression init,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation location = pp.getLocation();
		AnalysisState<A> assigned = state.bottom();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		MemoryAllocation allocation = new MemoryAllocation(dftype, location);
		AnalysisState<A> allocated = state.smallStepSemantics(allocation, pp);

		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			HeapReference ref = new HeapReference(dfref, loc, location);
			AnalysisState<A> readState = allocated.smallStepSemantics(init, pp).assign(loc, init, pp);
			assigned = readState.smallStepSemantics(ref, pp);
		}

		return assigned;
	}

	public static <A extends AbstractState<A>> AnalysisState<A> copyDataframe(
			AnalysisState<A> state,
			SymbolicExpression dataframe,
			ProgramPoint pp)
			throws SemanticException {
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		CodeLocation location = pp.getLocation();

		if (dataframe instanceof HeapReference || dataframe instanceof Variable)
			dataframe = new HeapDereference(dftype, dataframe, location);

		// we allocate the copy of the dataframe
		MemoryAllocation allocation = new MemoryAllocation(dftype, location);
		AnalysisState<A> allocated = state.smallStepSemantics(allocation, pp);

		AnalysisState<A> copy = state.bottom();
		UnaryExpression cp = new UnaryExpression(dftype, dataframe, new CopyDataframe(0), location);
		for (SymbolicExpression loc : allocated.getComputedExpressions())
			// copy the dataframe
			copy = copy.lub(allocated.smallStepSemantics(cp, pp).assign(loc, cp, pp));

		return copy;
	}

	public static <A extends AbstractState<A>> AnalysisState<A> applyUnary(
			AnalysisState<A> state,
			SymbolicExpression dataframe,
			ProgramPoint pp,
			UnaryOperator op)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> result = state.bottom();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);

		if (dataframe instanceof AccessChild)
			dataframe = ((AccessChild) dataframe).getContainer();

		// we allocate the copy that will contain the converted portion
		AnalysisState<A> copied = PandasSemantics.copyDataframe(state, dataframe, pp);

		for (SymbolicExpression id : copied.getComputedExpressions()) {
			// the new dataframe will receive the conversion
			UnaryExpression transform = new UnaryExpression(seriestype, id, op, loc);
			AnalysisState<A> tmp = copied.smallStepSemantics(transform, pp);

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(dfref, id, loc);
			result = result.lub(tmp.smallStepSemantics(ref, pp));
		}

		return result;
	}

	public static boolean isDataframePortionThatCanBeAssignedTo(
			SymbolicExpression left,
			ProgramPoint pp,
			SemanticOracle oracle) {
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (!(left instanceof HeapReference))
			return false;
		SymbolicExpression expression = ((HeapReference) left).getExpression();

		if (!(expression instanceof AccessChild))
			return false;

		Set<Type> rts = null;
		try {
			rts = oracle.getRuntimeTypesOf(expression, pp, oracle);
		} catch (SemanticException e) {
			return false;
		}

		if (rts == null || rts.isEmpty())
			return expression.getStaticType().equals(seriestype) || expression.getStaticType().equals(dftype);

		return rts.stream().anyMatch(t -> t.equals(seriestype) || t.equals(dftype));
	}

	public static HeapDereference getDataframeDereference(
			SymbolicExpression expr,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (isDataframePortionThatCanBeAssignedTo(expr, pp, oracle)) {
			HeapDereference firstDeref = (HeapDereference) ((AccessChild) ((HeapReference) expr).getExpression())
					.getContainer();

			if (!isDataframePortionThatCanBeAssignedTo(firstDeref.getExpression(), pp, oracle))
				return firstDeref;

			HeapDereference secondDereference = (HeapDereference) ((AccessChild) ((HeapReference) firstDeref
					.getExpression()).getExpression()).getContainer();
			return secondDereference;
		}

		throw new SemanticException("Access child expected");
	}

	public static <A extends AbstractState<A>> AnalysisState<A> compare(
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			ProgramPoint pp,
			SemanticOracle oracle,
			ComparisonOperator op)
			throws SemanticException {
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		Type seriesref = ((PyClassType) seriestype).getReference();
		if (oracle.getRuntimeTypesOf(left, pp, oracle).stream().anyMatch(t -> t.equals(seriesref))) {
			// custom behavior for comparison of expressions of the form
			// df["col1"] <= 4

			if (isDataframePortionThatCanBeAssignedTo(left, pp, oracle))
				left = getDataframeDereference(left, pp, oracle);

			// we allocate the copy that will contain the converted portion
			AnalysisState<A> copied = PandasSemantics.copyDataframe(state, left, pp);

			AnalysisState<A> result = state.bottom();
			CodeLocation loc = pp.getLocation();
			for (SymbolicExpression id : copied.getComputedExpressions()) {
				BinaryExpression seriesComp = new BinaryExpression(seriestype, id, right,
						new SeriesComparison(0, op), loc);
				AnalysisState<A> tmp = copied.smallStepSemantics(seriesComp, pp);

				HeapReference ref = new HeapReference(seriesref, id, loc);
				result = result.lub(tmp.smallStepSemantics(ref, pp));
			}
			return result;
		} else if (oracle.getRuntimeTypesOf(right, pp, oracle).stream().anyMatch(t -> t.equals(seriesref))) {
			// custom behavior for comparison of expressions of the form
			// 4 <= df["col1"]

			if (isDataframePortionThatCanBeAssignedTo(right, pp, oracle))
				right = getDataframeDereference(right, pp, oracle);

			// we allocate the copy that will contain the converted portion
			AnalysisState<A> copied = PandasSemantics.copyDataframe(state, right, pp);

			AnalysisState<A> result = state.bottom();
			CodeLocation loc = pp.getLocation();
			for (SymbolicExpression id : copied.getComputedExpressions()) {
				BinaryExpression seriesComp = new BinaryExpression(seriestype, id, left,
						new SeriesComparison(0, op), loc);
				AnalysisState<A> tmp = copied.smallStepSemantics(seriesComp, pp);

				HeapReference ref = new HeapReference(seriesref, id, loc);
				result = result.lub(tmp.smallStepSemantics(ref, pp));
			}
			return result;
		}
		return null;
	}
}
