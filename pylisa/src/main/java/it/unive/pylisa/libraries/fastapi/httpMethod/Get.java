package it.unive.pylisa.libraries.fastapi.httpMethod;

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

public class Get extends NaryExpression implements PluggableStatement {

    private Statement st;

    public Get(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "get", parameters);
    }

    @Override
    public void setOriginatingStatement(Statement st) { this.st = st; }

    @Override
    public Program getProgram() {
        return super.getProgram();
    }

    public static Get build(CFG cfg, CodeLocation location, Expression[] parameters) {
        return new Get(cfg, location, parameters);
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
