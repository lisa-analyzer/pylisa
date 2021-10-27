package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.call.BinaryNativeCall;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.UnsupportedStatementException;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class PySingleArrayAccess extends BinaryNativeCall {

    public PySingleArrayAccess(CFG cfg, CodeLocation loc, Type staticType, Expression receiver, Expression index) {
        super(cfg, loc, "[]", staticType, receiver, index);
    }

    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        return false;
    }


    @Override
    protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> binarySemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> leftState, SymbolicExpression leftExp, AnalysisState<A, H, V> rightState, SymbolicExpression rightExp) throws SemanticException {
        throw new SemanticException("Not yet supported");
    }
}
