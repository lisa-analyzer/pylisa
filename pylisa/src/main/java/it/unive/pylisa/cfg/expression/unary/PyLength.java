package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.symbolic.operators.value.StringLength;


public class PyLength extends it.unive.lisa.program.cfg.statement.UnaryExpression {
    
    public PyLength(CFG cfg, SourceCodeLocation location, Expression exp) {
        super(cfg, location, "len", exp);
    }
    
    @Override
    public <A extends AbstractState<A, H, V, T>,
            H extends HeapDomain<H>,
            V extends ValueDomain<V>,
            T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
            InterproceduralAnalysis<A, H, V, T> interprocedural,
            AnalysisState<A, H, V, T> state,
            SymbolicExpression expr,
            StatementStore<A, H, V, T> expressions)
            throws SemanticException {
        TypeSystem types = getProgram().getTypes();
        if (expr.getRuntimeTypes(types).stream().anyMatch(Type::isStringType)) {
            // String len
            return state.smallStepSemantics(
                    new UnaryExpression(Int32Type.INSTANCE, expr, StringLength.INSTANCE, getLocation()), this);
        }
        return state.bottom();
    }
    
    
}