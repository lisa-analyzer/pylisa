package it.unive.pylisa.libraries.experimental.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.libraries.fastapi.sarl.FastAPI;

public class HTTPEndpointStatement extends NaryExpression implements PluggableStatement {
    private Statement st;
    public HTTPEndpointStatement(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "HTTP endpoint", parameters);
    }

    public static HTTPEndpointStatement build(CFG cfg, CodeLocation location, Expression[] parameters) {
        return new HTTPEndpointStatement(cfg, location, parameters);
    }
    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A> interprocedural, AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
        return null;
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
}
