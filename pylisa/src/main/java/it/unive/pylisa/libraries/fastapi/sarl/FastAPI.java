package it.unive.pylisa.libraries.fastapi.sarl;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;

public class FastAPI extends NaryExpression implements PluggableStatement {

    private Statement st;

    public FastAPI(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "FastAPI", parameters);
    }

    @Override
    public void setOriginatingStatement(Statement st) { this.st = st; }

    @Override
    public Program getProgram() { return super.getProgram(); }

    public static FastAPI build(CFG cfg, CodeLocation location, Expression[] parameters) {
        return new FastAPI(cfg, location, parameters);
    }

    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
            InterproceduralAnalysis<A> interproceduralAnalysis,
            AnalysisState<A> analysisState,
            ExpressionSet[] expressionSets,
            StatementStore<A> statementStore) throws SemanticException {
        return null;
    }
}