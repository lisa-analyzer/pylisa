package it.unive.pylisa.libraries.fastapi.sarl.experimental;

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

public class HTTPCall extends NaryExpression implements PluggableStatement {

    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }
    public HTTPCall(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "HTTP", parameters);
    }
    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A> interprocedural, AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
        return null;
    }

    @Override
    public void setOriginatingStatement(Statement st) {

    }
}
