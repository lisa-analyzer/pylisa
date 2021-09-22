package it.unive.pylisa;


import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;

public class StupidConstruct extends NativeCall implements PluggableStatement {
    private Statement st;

    public StupidConstruct(CFG cfg, CodeLocation location, VariableRef constructName, Expression parameter) {
        super(cfg, location, "filterwarnings", parameter);
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

    @Override
    public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> callSemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V>[] computedStates, ExpressionSet<SymbolicExpression>[] params) throws SemanticException {
        return entryState;
    }
}