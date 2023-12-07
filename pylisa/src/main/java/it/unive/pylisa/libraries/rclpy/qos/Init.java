package it.unive.pylisa.libraries.rclpy.qos;

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
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyStringLiteral;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
    protected Statement st;

    public Init(CFG cfg, CodeLocation location, Expression[] exprs) {
        super(cfg, location, "__init__", exprs);
    }

    public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Init(cfg, location, exprs);
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

    @Override
    public String toString() {
        return "QosProfile";
    }

    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
            InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
            ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions)
            throws SemanticException {
        AnalysisState<A, H, V, T> result = state;
        Expression self = getSubExpressions()[0];
        NamedParameterExpression _avoid_ros_ns_conventions = getNamedParameterExpr("avoid_ros_namespace_convention");
        Expression avoid_ros_ns_conventions = _avoid_ros_ns_conventions != null
                ? _avoid_ros_ns_conventions.getSubExpression()
                : new FalseLiteral(this.getCFG(), getLocation());
        // _avoid_ros_ns_conventions.expressionSemantics(interprocedural, state, params,
        // expressions).getComputedExpressions();
        return result;
    }

    public NamedParameterExpression getNamedParameterExpr(String name) {
        for (Expression e : getSubExpressions()) {
            if (e instanceof NamedParameterExpression
                    && ((NamedParameterExpression) e).getParameterName().equals(name)) {
                return ((NamedParameterExpression) e);
            }
        }
        return null;
    }

}