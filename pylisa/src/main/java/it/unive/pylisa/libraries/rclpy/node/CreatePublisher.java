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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.NoOpFunction;

public class CreatePublisher extends NaryExpression implements PluggableStatement {
    protected Statement st;
    
    protected CreatePublisher(CFG cfg, CodeLocation location, String constructName,
                           Expression... parameters) {
        super(cfg, location, constructName, parameters);
    }
    
    public static CreatePublisher build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new CreatePublisher(cfg, location, "create_publisher", exprs);
    }
    
    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions) throws SemanticException {
        PushAny push = new PushAny(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER).getReference(),
                getLocation());
        return state.smallStepSemantics(push, st);
    }
    
    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
}
