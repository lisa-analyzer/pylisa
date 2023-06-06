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
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.NoOpFunction;
import org.apache.commons.lang3.ArrayUtils;

import javax.management.ValueExp;
import java.util.Arrays;
import java.util.Collections;

public class CreatePublisher extends NaryExpression implements PluggableStatement {
    protected Statement st;
    
    protected CreatePublisher(CFG cfg, CodeLocation location, String constructName,
                           Expression... parameters) {
        super(cfg, location, constructName, parameters);
    }
    
    public static CreatePublisher build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new CreatePublisher(cfg, location, "create_publisher", exprs);
    }

    /**
     *
     * @param interprocedural the interprocedural analysis of the program to
     *                            analyze
     * @param state           the state where the expression is to be evaluated
     * @param params          the symbolic expressions representing the computed
     *                            values of the sub-expressions of this
     *                            expression
     * @param expressions     the cache where analysis states of intermediate
     *                            expressions are stored and that can be
     *                            accessed to query for post-states of
     *                            parameters expressions
     *
     * @return
     * @param <A>
     * @param <H>
     * @param <V>
     * @param <T>
     * @throws SemanticException
     */
    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions) throws SemanticException {
        PyClassType publisherClassType = PyClassType.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER);

        PyNewObj publisherObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(), "__init__", publisherClassType, Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
        publisherObj.setOffset(st.getOffset());
        AnalysisState<A,H,V,T> newPublisherAS = publisherObj.expressionSemantics(interprocedural, state, params, expressions);
        state =  state.lub(newPublisherAS);
        // get _publishers list
        AccessInstanceGlobal aig = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0], "_publisher");
        AnalysisState<A,H,V,T> aigSemantics = aig.semantics(state, interprocedural, expressions);
        for (SymbolicExpression e: aigSemantics.getComputedExpressions()) {
            ExpressionSet<ValueExpression> ves = state.getState().getHeapState().rewrite(e, st);
            for (ValueExpression ve: ves) {
                System.out.println("e");
            }
        }
        return state;
    }
    
    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
}
