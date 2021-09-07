package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;

import java.util.Arrays;
import java.util.List;

public class PyArrayAccess extends Expression {
    private final List<Expression> indexes;
    private final Expression receiver;

    public PyArrayAccess(Expression receiver, List<Expression> indexes, CFG cfg, CodeLocation loc) {
        super(cfg, loc);
        this.receiver = receiver;
        this.indexes = indexes;
    }

    @Override
    public int setOffset(int i) {
        super.offset = i;
        return i;
    }

    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        return false;
    }

    @Override
    public String toString() {
        return receiver.toString()+"["+ Arrays.toString(indexes.toArray())+"]";
    }

    @Override
    public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> semantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, StatementStore<A, H, V> expressions) throws SemanticException {
        throw new SemanticException("Not yet supported");
    }


}
