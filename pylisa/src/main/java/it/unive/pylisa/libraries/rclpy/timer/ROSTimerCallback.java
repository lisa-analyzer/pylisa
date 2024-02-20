package it.unive.pylisa.libraries.rclpy.timer;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;

public class ROSTimerCallback extends NaryExpression {
    public ROSTimerCallback(CFG cfg, SourceCodeLocation location, Expression subExpression) {
        super(cfg, location, "ros_timer_callback", subExpression);
    }

    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }

    public <A extends AbstractState<A>> AnalysisState<A> snooping(InterproceduralAnalysis<A> interprocedural, AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
        for (SymbolicExpression expr : params[0]) {
            if (expr instanceof AccessChild) {
                AccessChild access = (AccessChild) expr;
                SymbolicExpression container = access.getContainer();
                SymbolicExpression child = access.getChild();
                if (container instanceof HeapDereference) {
                    HeapDereference containerDeref = (HeapDereference) container;
                    HeapReference heapRef = new HeapReference(containerDeref.getStaticType(), containerDeref, containerDeref.getCodeLocation());
                    ExpressionSet exprSet = state.getState().rewrite(heapRef, this, state.getState());
                    VariableRef selfRef = new VariableRef(this.getCFG(), containerDeref.getCodeLocation(), containerDeref.getExpression().toString());
                    Expression[] unresolvedCallExprs = new Expression[1];
                    unresolvedCallExprs[0] = selfRef;
                    UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.UNKNOWN, null, child.toString(), unresolvedCallExprs);
                    SymbolicExpression selfRefSE = new Variable(exprSet.iterator().next().getStaticType(), selfRef.getName(), selfRef.getLocation());
                    ExpressionSet[] unresolvedCallParams = new ExpressionSet[1];
                    unresolvedCallParams[0] = new ExpressionSet(selfRefSE);

                    unresolvedCall.forwardSemanticsAux(interprocedural, state, unresolvedCallParams, expressions);
                }

            }
            if (expr instanceof ValueExpression) {
                Expression[] unresolvedCallExprs = new Expression[0];
                UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.STATIC, null, expr.toString(), unresolvedCallExprs);
                ExpressionSet[] unresolvedCallParams = new ExpressionSet[0];
                unresolvedCall.forwardSemanticsAux(interprocedural, state, unresolvedCallParams, expressions);
            }
        }
        return state;
    }


    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A> interprocedural, AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
        SymbolicExpression self = params[0].iterator().next();
        ExpressionSet exprSet = state.getState().rewrite(self, this, state.getState());
        var result = state.bottom();
        Expression e = getSubExpressions()[0];
        if (e instanceof AccessInstanceGlobal) {
            AccessInstanceGlobal aig = (AccessInstanceGlobal) e;
            String target = aig.getTarget();
            VariableRef selfRef = (VariableRef) aig.getSubExpressions()[0];
            Expression[] unresolvedCallExprs = new Expression[2];
            unresolvedCallExprs[0] = selfRef;
            UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.UNKNOWN, null, target, unresolvedCallExprs);
            unresolvedCall.forwardSemanticsAux(interprocedural, state, params, expressions);

        }
        return state;
    }
}
