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
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;
import it.unive.pylisa.symbolic.operators.PandasSeriesComparison;

public class PandasSeriesComparisonSemantics {
    public static<A extends AbstractState<A, H, V, T> , H extends HeapDomain<H> , V extends ValueDomain<V> , T extends TypeDomain<T>> AnalysisState<A, H, V, T> pandasSeriesBinarySemantics(
        InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
        SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V, T> expressions, ProgramPoint pp, CodeLocation location, ComparisonOperator op)
    throws SemanticException {
        if (left.getRuntimeTypes().anyMatch(t -> t.equals(PandasSeriesType.REFERENCE)) &&
            right.getRuntimeTypes().anyMatch(t -> t.isNumericType() || t.isStringType())) {
            // custom behavior for comparison of expressions of the form df["col1"] <= 4

            BinaryExpression seriesComp = new BinaryExpression(PandasSeriesType.INSTANCE, left, right, new PandasSeriesComparison(op), location);
            return state.smallStepSemantics(seriesComp, pp);
        }
        return null;
    }
}