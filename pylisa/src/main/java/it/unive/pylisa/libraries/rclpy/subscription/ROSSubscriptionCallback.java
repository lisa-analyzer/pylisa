package it.unive.pylisa.libraries.rclpy.subscription;

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
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;

import java.util.HashSet;
import java.util.Set;

public class ROSSubscriptionCallback extends NaryExpression {
    public ROSSubscriptionCallback(CFG cfg, SourceCodeLocation location, Expression subExpression) {
        super(cfg, location, "ros_callback", subExpression);
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
                    SymbolicExpression self = ((HeapDereference) container).getExpression();
                    VariableRef message = new VariableRef(this.getCFG(), this.getLocation(), "$msg");
                    VariableRef selfRef = new VariableRef(this.getCFG(), containerDeref.getCodeLocation(), containerDeref.getExpression().toString());
                    Expression[] unresolvedCallExprs = new Expression[2];
                    unresolvedCallExprs[0] = selfRef;
                    unresolvedCallExprs[1] = message;
                    UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.UNKNOWN, null, child.toString(), unresolvedCallExprs);
                    SymbolicExpression se = new Variable(Untyped.INSTANCE, "$msg", SyntheticLocation.INSTANCE);
                    SymbolicExpression selfRefSE = new Variable(exprSet.iterator().next().getStaticType(), selfRef.getName(), selfRef.getLocation());
                    ExpressionSet[] unresolvedCallParams = new ExpressionSet[2];
                    unresolvedCallParams[0] = new ExpressionSet(selfRefSE);
                    unresolvedCallParams[1] = new ExpressionSet(se);

                    unresolvedCall.forwardSemanticsAux(interprocedural, state, unresolvedCallParams, expressions);
                    var x = 3;
                }
                var x = 3;

            }
            if (expr instanceof ValueExpression) {
                VariableRef message = new VariableRef(this.getCFG(), this.getLocation(), "$msg");
                Expression[] unresolvedCallExprs = new Expression[1];
                unresolvedCallExprs[0] = message;
                UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.STATIC, null, expr.toString(), unresolvedCallExprs);
                SymbolicExpression se = new Variable(Untyped.INSTANCE, "$msg", SyntheticLocation.INSTANCE);
                ExpressionSet[] unresolvedCallParams = new ExpressionSet[1];
                unresolvedCallParams[0] = new ExpressionSet(se);
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
            VariableRef message = new VariableRef(this.getCFG(), this.getLocation(), "$msg");
            VariableRef selfRef = (VariableRef) aig.getSubExpressions()[0];
            Expression[] unresolvedCallExprs = new Expression[2];
            unresolvedCallExprs[0] = selfRef;
            unresolvedCallExprs[1] = message;
            UnresolvedCall unresolvedCall = new UnresolvedCall(this.getCFG(), this.getLocation(), Call.CallType.UNKNOWN, null, target, unresolvedCallExprs);
            unresolvedCall.forwardSemanticsAux(interprocedural, state, params, expressions);

        }
        return state;
    }
}
