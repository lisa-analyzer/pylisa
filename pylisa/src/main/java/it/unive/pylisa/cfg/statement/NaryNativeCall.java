package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;

import java.util.HashSet;
import java.util.Set;

public abstract class NaryNativeCall extends NativeCall {
    protected NaryNativeCall(CFG cfg, CodeLocation location, String constructName, Expression... parameters) {
        super(cfg, location, constructName, parameters);
    }

    protected NaryNativeCall(CFG cfg, CodeLocation location, String constructName, Type staticType, Expression... parameters) {
        super(cfg, location, constructName, staticType, parameters);
    }

    @Override
    public <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> callSemantics(AnalysisState<A, H, V> entryState, InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V>[] computedStates, ExpressionSet<SymbolicExpression>[] params) throws SemanticException {
        Set<SymbolicExpression[]> transformedExpressions = transform(params);
        AnalysisState<A, H, V> result = entryState.bottom();

        for (SymbolicExpression[] exprs : transformedExpressions) {
                AnalysisState<A, H, V> tmp = narySemantics(entryState, interprocedural, computedStates, exprs);
                result = result.lub(tmp);
        }
        return result;
    }
    /**
     * Computes the semantics of the call, after the semantics of the parameters
     * have been computed. Meta variables from the parameters will be forgotten
     * after this call returns.
     *
     * @param <A>             the type of {@link AbstractState}
     * @param <H>             the type of the {@link HeapDomain}
     * @param <V>             the type of the {@link ValueDomain}
     * @param entryState      the entry state of this binary call
     * @param interprocedural the interprocedural analysis of the program to
     *                            analyze
     * @param computedStates  the array of states chaining the parameters'
     *                            semantics evaluation starting from
     *                            {@code entryState}, namely
     *                            {@code computedState[i]} corresponds to the
     *                            state obtained by the evaluation of
     *                            {@code params[i]} in the state
     *                            {@code computedState[i-1]} ({@code params[0]}
     *                            is evaluated in {@code entryState})
     * @param exprs          the symbolic expressions representing the computed
     *                            values of the parameters of this call
     *
     * @return the {@link AnalysisState} representing the abstract result of the
     *             execution of this call
     *
     * @throws SemanticException if something goes wrong during the computation
     */
    public abstract <A extends AbstractState<A, H, V>, V extends ValueDomain<V>, H extends HeapDomain<H>> AnalysisState<A,H,V> narySemantics(
            AnalysisState<A,H,V> entryState,
            InterproceduralAnalysis<A,H,V> interprocedural,
            AnalysisState<A,H,V>[] computedStates,
            SymbolicExpression[] exprs);

    private Set<SymbolicExpression[]> transform(ExpressionSet<SymbolicExpression>[] params) {
        Set<SymbolicExpression[]> result = new HashSet<>();
        for(SymbolicExpression expr : params[0]) {
            SymbolicExpression[] value = new SymbolicExpression[1];
            value[0]=expr;
            result.add(value);
        }
        for(int i=1; i<params.length; i++) {
            Set<SymbolicExpression[]> old = result;
            result = new HashSet<>();
            for(SymbolicExpression expr : params[i])
                for(SymbolicExpression[] oldarray : old) {
                    SymbolicExpression[] value = new SymbolicExpression[oldarray.length+1];
                    value[i] = expr;
                    result.add(value);
            }
        }
        return result;
    }
}
