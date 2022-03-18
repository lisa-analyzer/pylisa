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
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.PopSelection;

public class PandasSemantics {

	private PandasSemantics() {
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> copyDataframe(
					AnalysisState<A, H, V, T> state,
					SymbolicExpression dataframe,
					ProgramPoint pp)
					throws SemanticException {

		CodeLocation location = pp.getLocation();
		if (!(dataframe instanceof HeapDereference))
			dataframe = new HeapDereference(PandasDataframeType.INSTANCE, dataframe, location);

		// we allocate the copy of the dataframe
		HeapAllocation allocation = new HeapAllocation(PandasDataframeType.INSTANCE, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, pp);

		AnalysisState<A, H, V, T> copy = state.bottom();
		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			// copy the dataframe
			AnalysisState<A, H, V, T> assigned = allocated.assign(loc, dataframe, pp);
			copy = copy.lub(assigned);
		}

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

		if (dataframe instanceof AccessChild)
			dataframe = ((AccessChild) dataframe).getContainer();

		// we allocate the copy that will contain the converted portion
		AnalysisState<A, H, V, T> copied = PandasSemantics.copyDataframe(state, dataframe, pp);

		UnaryExpression pop = new UnaryExpression(PandasDataframeType.INSTANCE, dataframe, PopSelection.INSTANCE, loc);
		for (SymbolicExpression id : copied.getComputedExpressions()) {
			// the new dataframe will receive the conversion
			UnaryExpression transform = new UnaryExpression(PandasSeriesType.INSTANCE, id, op, loc);
			AnalysisState<A, H, V, T> tmp = copied.smallStepSemantics(transform, pp);

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(PandasDataframeType.REFERENCE, id, loc);
			result = result.lub(tmp.smallStepSemantics(pop, pp).smallStepSemantics(ref, pp));
		}

		return result;
	}

	public static SymbolicExpression getDataframeDereference(SymbolicExpression accessChild) {
		if (accessChild instanceof AccessChild 
			&& accessChild.getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.REFERENCE) || t.equals(PandasSeriesType.REFERENCE))) {
			HeapDereference firstDeref = (HeapDereference) ((AccessChild) accessChild).getContainer();
			
			if (!(firstDeref.getExpression() instanceof AccessChild)) {
				return firstDeref;
			}

			HeapDereference secondDereference = (HeapDereference) ((AccessChild) firstDeref.getExpression()).getContainer();
			return secondDereference;
		}
		return accessChild;
	}
}
