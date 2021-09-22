package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.caches.Caches;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.BitExternalSet;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.lisa.util.collections.externalSet.ExternalSetCache;
import it.unive.lisa.util.collections.externalSet.UniversalExternalSet;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.type.PyLibraryType;
import it.unive.pylisa.symbolic.LibraryIdentifier;

import java.util.HashSet;
import java.util.Set;

public class Import extends Statement {

    protected final String importedLibrary;
    protected final String name;

    //import <importedLibrary> as <name>
    public Import(String importedLibrary, String name, CFG cfg, CodeLocation loc) {
        super(cfg, loc);
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
        return visitor.visit(tool, getCFG(), this);
    }

    @Override
    public String toString() {
        return "import "+importedLibrary+" as "+name;
    }

    @Override
    public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> semantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, StatementStore<A, H, V> expressions) throws SemanticException {
        SymbolicExpression libexpr = new LibraryIdentifier(importedLibrary, this.getLocation());
        Variable var = new Variable(Caches.types().mkSingletonSet(new PyLibraryType(importedLibrary)), name, this.getLocation());
        return entryState.assign(var, libexpr, this);
    }

}
