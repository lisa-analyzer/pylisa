package it.unive.pylisa.libraries.miniapi;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;

public class MiniAPI extends UnaryExpression implements PluggableStatement {

    private Statement st;

    public MiniAPI(CFG cfg, CodeLocation location, String constructName, Expression expr) {
        super(cfg, location, "MiniAPI", expr);
    }

    @Override
    protected int compareSameClassAndParams(Statement o) { return 0; }

    @Override
    public void setOriginatingStatement(Statement st) { this.st = st; }

    public static MiniAPI build(CFG cfg, CodeLocation location, String constructName, Expression expr) {
        return new MiniAPI(cfg, location, "MiniAPI", expr);
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
            InterproceduralAnalysis<A> interprocedural,
            AnalysisState<A> state,
            SymbolicExpression arg,
            StatementStore<A> expressions) throws SemanticException {
        return state;
    }
}