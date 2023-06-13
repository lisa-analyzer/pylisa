package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Set;

/*
    https://docs.python.org/3/library/functions.html#super
 */
public class SimpleSuperUnresolvedCall extends UnresolvedCall {
    UnresolvedCall call;

    public SimpleSuperUnresolvedCall(CFG cfg, CodeLocation location, CallType callType, String qualifier, String targetName,
                                     EvaluationOrder order, Type staticType, Expression... parameters) {
        super(cfg, location, callType, qualifier, targetName, order, staticType, parameters);
        Expression[] expr = new Expression[0];
        call = new UnresolvedCall(cfg, location, callType, qualifier, targetName, order, staticType, expr);
    }


    public SimpleSuperUnresolvedCall(CFG cfg, CodeLocation location, CallType callType, String qualifier, String targetName,
                                     Expression... parameters) {
        this(cfg, location, callType, qualifier, targetName, Untyped.INSTANCE, parameters);
    }


    public SimpleSuperUnresolvedCall(CFG cfg, CodeLocation location, CallType callType, String qualifier, String targetName,
                                     Type staticType, Expression... parameters) {
        this(cfg, location, callType, qualifier, targetName, LeftToRightEvaluation.INSTANCE, staticType, parameters);
    }


    public SimpleSuperUnresolvedCall(CFG cfg, CodeLocation location, CallType callType, String qualifier, String targetName,
                                     EvaluationOrder order, Expression... parameters) {
        this(cfg, location, callType, qualifier, targetName, order, Untyped.INSTANCE, parameters);
    }


    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions) throws SemanticException {
        Call resolved;
        @SuppressWarnings("unchecked")
		Set<Type>[] types = new Set[0];
        try {
            resolved = interprocedural.resolve(call, types, state.getAliasing());
            if (resolved instanceof OpenCall) {

                resolved = interprocedural.resolve(this, parameterTypes(expressions), state.getAliasing());
            }
        } catch (CallResolutionException e) {
            throw new SemanticException("Unable to resolve call " + this, e);
        }

        AnalysisState<A, H, V, T> result = resolved.expressionSemantics(interprocedural, state, params, expressions);
        getMetaVariables().addAll(resolved.getMetaVariables());
        return result;
    }
}
