package it.unive.pylisa.analysis.libraries.pandas;


import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.BinaryNativeCall;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyDataframeType;
import it.unive.pylisa.symbolic.DataFrameConstant;

public class ReadCsv extends BinaryNativeCall implements PluggableStatement {
    private Statement st;

    protected ReadCsv(CFG cfg, CodeLocation location, String constructName, Type staticType, Expression left, Expression right) {
        super(cfg, location, constructName, staticType, left, right);
    }

    @Override
    final public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

    @Override
    protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> binarySemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> leftState, SymbolicExpression leftExp, AnalysisState<A, H, V> rightState, SymbolicExpression rightExp) throws SemanticException {
        return rightState.smallStepSemantics(new DataFrameConstant(PyDataframeType.INSTANCE, rightExp, this.getLocation()), st);
    }

    public static NativeCall build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new ReadCsv(cfg, location, "read_csv", PyDataframeType.INSTANCE, exprs[0], exprs[1]);
    }
}