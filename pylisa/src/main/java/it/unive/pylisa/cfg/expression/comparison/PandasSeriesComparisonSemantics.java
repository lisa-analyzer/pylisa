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
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.DataframeColumnComparison;

public class PandasSeriesComparisonSemantics {
    public static<A extends AbstractState<A, H, V, T> , H extends HeapDomain<H> , V extends ValueDomain<V> , T extends TypeDomain<T>> AnalysisState<A, H, V, T> pandasSeriesBinarySemantics(
        InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
        SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V, T> expressions, ProgramPoint pp, CodeLocation location, DataframeColumnComparison.Operator op)
    throws SemanticException {
        if (left.getRuntimeTypes().anyMatch(t -> t.equals(PandasSeriesType.REFERENCE)) &&
            right.getRuntimeTypes().anyMatch(Type::isNumericType)) {
            // custom behavior for comparison of expressions of the form df["col1"] <= 4
            if (left instanceof BinaryExpression) {
                BinaryExpression leftBin = (BinaryExpression) left;
                if (leftBin.getOperator() instanceof ColumnAccess &&
                    leftBin.getLeft().getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.INSTANCE)) &&
                    leftBin.getRight().getRuntimeTypes().anyMatch(Type::isStringType) &&
                    right.getRuntimeTypes().anyMatch(t -> t.isNumericType() || t.isStringType())) {
                    Constant colAccessed = (Constant) leftBin.getRight();
                    Constant value = (Constant) right;

                    TernaryExpression seriesComp = new TernaryExpression(PandasSeriesType.INSTANCE, leftBin.getLeft(), colAccessed, value, new DataframeColumnComparison(op), location);
                    return state.smallStepSemantics(seriesComp, pp);
                }
            }
        }
        return null;
    }
}