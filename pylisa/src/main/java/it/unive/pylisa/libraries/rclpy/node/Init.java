package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAssign;
import java.util.Collection;


public class Init extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {
    protected Statement st;
    public Init(CFG cfg, CodeLocation location, Expression first, Expression second) {
        super(cfg, location, "init", first, second);
    }

    public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Init(cfg, location, exprs[0], exprs[1]);
    }


    @Override
    public String toString() {
        return "__init__";
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V, T> expressions) throws SemanticException {
        AnalysisState<A,H,V,T> result = state;
        ExpressionSet<SymbolicExpression>[] expr = new ExpressionSet[0];

        AccessInstanceGlobal nodeName = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0], "node_name"); // super(MinimalSubscriber, self)::node_name
        PyAssign pyAssign = new PyAssign(getCFG(), getLocation(), nodeName, getSubExpressions()[1]);
        result = result.lub(pyAssign.semantics(result, interprocedural, expressions));
        // new empy List of Publishers
        AccessInstanceGlobal _publishers = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0], "_publishers");
        ListCreation _publishersList = new ListCreation(getCFG(), getLocation());
        pyAssign = new PyAssign(getCFG(), getLocation(), _publishers, _publishersList);
        result = result.lub(pyAssign.semantics(result, interprocedural, expressions));

        // new empty list of Subscribers
        AccessInstanceGlobal _subscribers = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0], "_subscribers");
        ListCreation _subscribersList = new ListCreation(getCFG(), getLocation());
        pyAssign = new PyAssign(getCFG(), getLocation(), _subscribers, _subscribersList);
        result = result.lub(pyAssign.semantics(result, interprocedural, expressions));

        return result.lub(pyAssign.semantics(result, interprocedural, expressions));
    }
}