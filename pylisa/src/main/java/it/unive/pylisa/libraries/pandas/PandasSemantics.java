package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.ComparisonOperator;
import it.unive.pylisa.symbolic.operators.dataframes.CopyDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.PandasSeriesComparison;

public class PandasSemantics {

	private PandasSemantics() {
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> createAndInitDataframe(
					AnalysisState<A, H, V, T> state,
					SymbolicExpression init,
					ProgramPoint pp)
					throws SemanticException {
		CodeLocation location = pp.getLocation();
		AnalysisState<A, H, V, T> assigned = state.bottom();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		HeapAllocation allocation = new HeapAllocation(dftype, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, pp);

		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			HeapReference ref = new HeapReference(dfref, loc, location);
			AnalysisState<A, H, V, T> readState = allocated.smallStepSemantics(init, pp).assign(loc, init, pp);
			assigned = readState.smallStepSemantics(ref, pp);
		}

		return assigned;
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> copyDataframe(
					AnalysisState<A, H, V, T> state,
					SymbolicExpression dataframe,
					ProgramPoint pp)
					throws SemanticException {
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		CodeLocation location = pp.getLocation();

		if (!(dataframe instanceof HeapDereference))
			dataframe = new HeapDereference(dftype, dataframe, location);

		// we allocate the copy of the dataframe
		HeapAllocation allocation = new HeapAllocation(dftype, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, pp);

		AnalysisState<A, H, V, T> copy = state.bottom();
		UnaryExpression cp = new UnaryExpression(dftype, dataframe, CopyDataframe.INSTANCE, location);
		for (SymbolicExpression loc : allocated.getComputedExpressions())
			// copy the dataframe
			copy = copy.lub(allocated.smallStepSemantics(cp, pp).assign(loc, cp, pp));

		return copy;
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> transform(
					AnalysisState<A, H, V, T> state,
					SymbolicExpression dataframe,
					ProgramPoint pp,
					ApplyTransformation op)
					throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A, H, V, T> result = state.bottom();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);

		if (dataframe instanceof AccessChild)
			dataframe = ((AccessChild) dataframe).getContainer();

		// we allocate the copy that will contain the converted portion
		AnalysisState<A, H, V, T> copied = PandasSemantics.copyDataframe(state, dataframe, pp);

		for (SymbolicExpression id : copied.getComputedExpressions()) {
			// the new dataframe will receive the conversion
			UnaryExpression transform = new UnaryExpression(seriestype, id, op, loc);
			AnalysisState<A, H, V, T> tmp = copied.smallStepSemantics(transform, pp);

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(dfref, id, loc);
			result = result.lub(tmp.smallStepSemantics(ref, pp));
		}

		return result;
	}

	public static HeapDereference getDataframeDereference(SymbolicExpression expr) throws SemanticException {
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		Type seriesref = ((PyClassType) seriestype).getReference();

		if (expr instanceof AccessChild
				&& expr.getRuntimeTypes().anyMatch(t -> t.equals(dfref) || t.equals(seriesref))) {
			HeapDereference firstDeref = (HeapDereference) ((AccessChild) expr).getContainer();

			if (!(firstDeref.getExpression() instanceof AccessChild))
				return firstDeref;

			HeapDereference secondDereference = (HeapDereference) ((AccessChild) firstDeref.getExpression())
					.getContainer();
			return secondDereference;
		}

		throw new SemanticException("Access child expected");
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> compare(
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					ProgramPoint pp,
					ComparisonOperator op)
					throws SemanticException {
		PyClassType type = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		Type typeref = ((PyClassType) type).getReference();
		if (left.getRuntimeTypes().anyMatch(t -> t.equals(typeref)) &&
				right.getRuntimeTypes().anyMatch(t -> t.isNumericType() || t.isStringType())) {
			// custom behavior for comparison of expressions of the form
			// df["col1"] <= 4

			if (left instanceof AccessChild)
				left = getDataframeDereference(left);

			BinaryExpression seriesComp = new BinaryExpression(type, left, right,
					new PandasSeriesComparison(op), pp.getLocation());
			return state.smallStepSemantics(seriesComp, pp);
		}
		return null;
	}
}
