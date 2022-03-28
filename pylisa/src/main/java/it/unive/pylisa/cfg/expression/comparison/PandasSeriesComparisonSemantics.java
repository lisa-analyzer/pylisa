package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;
import it.unive.pylisa.symbolic.operators.PandasSeriesComparison;

public class PandasSeriesComparisonSemantics {
	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> pandasSeriesBinarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions,
					ProgramPoint pp,
					CodeLocation location,
					ComparisonOperator op)
					throws SemanticException {
		PyClassType type = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		Type typeref = ((PyClassType) type).getReference();
		if (left.getRuntimeTypes().anyMatch(t -> t.equals(typeref)) &&
				right.getRuntimeTypes().anyMatch(t -> t.isNumericType() || t.isStringType())) {
			// custom behavior for comparison of expressions of the form
			// df["col1"] <= 4

			SymbolicExpression dfDereference = PandasSemantics.getDataframeDereference(left);

			BinaryExpression seriesComp = new BinaryExpression(type, dfDereference, right,
					new PandasSeriesComparison(op), location);
			return state.smallStepSemantics(seriesComp, pp);
		}
		return null;
	}
}