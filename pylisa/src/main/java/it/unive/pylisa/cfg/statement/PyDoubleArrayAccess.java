package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NativeCall;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.TernaryNativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;

public class PyDoubleArrayAccess extends TernaryNativeCall {

    public PyDoubleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index1, Expression index2) {
        super(cfg, loc, "[]", staticType, receiver, index1, index2);
    }

    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        return false;
    }


    @Override
    protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> ternarySemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> leftState, SymbolicExpression leftExp, AnalysisState<A, H, V> middleState, SymbolicExpression middleExp, AnalysisState<A, H, V> rightState, SymbolicExpression rightExp) throws SemanticException {
        throw new SemanticException("Not yer supported");
    }
}
