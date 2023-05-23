package it.unive.pylisa.libraries.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import it.unive.lisa.type.*;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.symbolic.operators.value.StringConstructor;

import java.util.Collection;


public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
    protected Statement st;

    public Init(CFG cfg, CodeLocation location, Expression[] exprs) {
        super(cfg, location, "init", exprs);
    }

    public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Init(cfg, location, exprs);
    }


    @Override
    public String toString() {
        return "__init__";
    }

    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions) throws SemanticException {
        return state;
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

}