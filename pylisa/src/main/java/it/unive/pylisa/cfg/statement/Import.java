package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallGraph;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.UnsupportedStatementException;

public class Import extends Statement {

    protected String importedLibrary, name;

    //import <importedLibrary> as <name>
    public Import(String importedLibrary, String name, CFG cfg, CodeLocation loc) {
        super(cfg, loc);
        this.importedLibrary = importedLibrary;
        this.name = name;
    }

    //import <importedLibrary>
    public Import(String importedLibrary, CFG cfg, CodeLocation loc) {
        this(importedLibrary, importedLibrary, cfg, loc);
        this.importedLibrary = importedLibrary;
        this.name = name;
    }
    @Override
    public int setOffset(int i) {
        super.offset = i;
        return i;
    }

    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        throw new UnsupportedStatementException();
    }

    @Override
    public String toString() {
        return "import "+importedLibrary+" as "+name;
    }

    @Override
    public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> semantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, StatementStore<A, H, V> expressions) throws SemanticException {
        return entryState;
    }

}
