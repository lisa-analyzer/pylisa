package it.unive.pylisa.libraries.fastapi.sarl.method;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;

public class Delete extends NaryExpression implements PluggableStatement {

    private Statement st;

    public Delete(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "delete", parameters);
    }

    @Override
    public void setOriginatingStatement(Statement st) { this.st = st; }

    @Override
    public Program getProgram() { return super.getProgram(); }

    public static Delete build(CFG cfg, CodeLocation location, Expression[] parameters) {
        return new Delete(cfg, location, parameters);
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
