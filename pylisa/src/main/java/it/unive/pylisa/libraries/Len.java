package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.StringLength;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;


public class Len extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
    protected Statement st;

    protected Len(CFG cfg, CodeLocation location, String constructName,
                  Expression sequence) {
        super(cfg, location, constructName, sequence);
    }

    public static Len build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Len(cfg, location, "len", exprs[0]);
    }


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
        // TODO handle other cases
        return state;
    }

    @Override
    public String toString() {
        return "len";
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
}