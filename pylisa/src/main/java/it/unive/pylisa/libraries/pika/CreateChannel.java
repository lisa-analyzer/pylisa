package it.unive.pylisa.libraries.pika;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.ros.lisa.symbolic.operators.ros.ROSTopicNameExpansion;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class CreateChannel extends NaryExpression implements PluggableStatement {
    protected Statement st;

    protected CreateChannel(
            CFG cfg,
            CodeLocation location,
            String constructName,
            Expression... parameters) {
        super(cfg, location, constructName, parameters);
    }

    @Override
    protected int compareSameClassAndParams(
            Statement o) {
        return 0;
    }

    public static CreateChannel build(
            CFG cfg,
            CodeLocation location,
            Expression[] exprs) {
        return new CreateChannel(cfg, location, "create_channel", exprs);
    }

    /**
     * @param interprocedural the interprocedural analysis of the program to
     *                        analyze
     * @param state           the state where the expression is to be evaluated
     * @param params          the symbolic expressions representing the computed
     *                        values of the sub-expressions of this
     *                        expression
     * @param expressions     the cache where analysis states of intermediate
     *                        expressions are stored and that can be
     *                        accessed to query for post-states of
     *                        parameters expressions
     *
     * @return
     *
     * @param <A>
     * @param <H>
     * @param <V>
     * @param <T>
     *
     * @throws SemanticException
     */
    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
            InterproceduralAnalysis<A> interprocedural,
            AnalysisState<A> state,
            ExpressionSet[] params,
            StatementStore<A> expressions)
            throws SemanticException {
        AnalysisState<A> result = state.bottom();


        PyClassType channelClassType = PyClassType.lookup("pika.Channel");

        PyNewObj channelObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(), "__init__",
                channelClassType);
        AnalysisState<A> channelAS = channelObj.forwardSemanticsAux(interprocedural,
                state, params, expressions);

        result =  result.lub(channelAS);

        return result;
    }

    @Override
    public void setOriginatingStatement(
            Statement st) {
        this.st = st;
    }
}
