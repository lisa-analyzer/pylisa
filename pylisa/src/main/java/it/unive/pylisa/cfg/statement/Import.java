package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.impl.types.TypeEnvironment;
import it.unive.lisa.callgraph.CallGraph;
import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Statement;

public class Import extends Statement {

    protected String importedLibrary, name;

    //import <importedLibrary> as <name>
    public Import(String importedLibrary, String name, CFG cfg, String sourceFile, int line, int col) {
        super(cfg, sourceFile, line, col);
        this.importedLibrary = importedLibrary;
        this.name = name;
    }

    //import <importedLibrary>
    public Import(String importedLibrary, CFG cfg, String sourceFile, int line, int col) {
        this(importedLibrary, importedLibrary, cfg, sourceFile, line, col);
        this.importedLibrary = importedLibrary;
        this.name = name;
    }
    @Override
    public int setOffset(int i) {
        super.offset = i;
        return i;
    }

    @Override
    public String toString() {
        return "import "+importedLibrary+" as "+name;
    }

    @Override
    public <H extends HeapDomain<H>> AnalysisState<H, TypeEnvironment> typeInference(AnalysisState<H, TypeEnvironment> analysisState, CallGraph callGraph, ExpressionStore<AnalysisState<H, TypeEnvironment>> expressionStore) throws SemanticException {
        return analysisState;
    }

    @Override
    public <H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<H, V> semantics(AnalysisState<H, V> analysisState, CallGraph callGraph, ExpressionStore<AnalysisState<H, V>> expressionStore) throws SemanticException {
        //FIXME: for now it does nothing
        return analysisState;
    }
}
