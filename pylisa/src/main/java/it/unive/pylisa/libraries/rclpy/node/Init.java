package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import it.unive.lisa.type.*;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.symbolic.operators.value.StringConstructor;

import java.util.Collection;


public class Init extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {
    protected Statement st;

    public Init(CFG cfg, CodeLocation location, Expression first, Expression second) {
        super(cfg, location, "init", first, second);
    }

    public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Init(cfg, location, exprs[0], exprs[1]);
    }


    @Override
    public String toString() {
        return "__init__";
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }

    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V, T> expressions) throws SemanticException {
        VariableTableEntry callerReceiverVte = st.getCFG().getDescriptor().getVariables().get(0);
        String callerReceiverName = callerReceiverVte.getName(); // the first variable is the receiver
        CodeLocation callerReceiverLocation = callerReceiverVte.getLocation();
        AnalysisState<A,H,V,T> result = state.bottom();
        for (SymbolicExpression eLeft : expressions.getState(getLeft()).getComputedExpressions()) {
            for (SymbolicExpression eRight: expressions.getState(getRight()).getComputedExpressions()) {
                    AnalysisState<A, H, V, T> innerState = result.bottom();
                    //HeapReference ref = new HeapReference(callerReceiverVte.getStaticType(), loc, getLocation());
                    HeapReference ref = new HeapReference(callerReceiverVte.getStaticType(), eLeft, getLocation());
                    HeapDereference deref = new HeapDereference(callerReceiverVte.getStaticType(), ref, getLocation());

                    Variable node_name = new Variable(StringType.INSTANCE, "node_name",
                            getLocation());
                    AccessChild accessChild = new AccessChild(StringType.INSTANCE, deref,
                            node_name, getLocation());
                    innerState = result.smallStepSemantics(accessChild, this);
                    AnalysisState<A, H, V, T> _innerState = result.bottom();
                    for (SymbolicExpression lenId : innerState.getComputedExpressions())
                        _innerState = innerState.lub(innerState.assign(lenId, new Constant(StringType.INSTANCE, "TEST", getLocation()), this));
                    //result.lub(state.smallStepSemantics(new AccessChild(callerReceiverVte.getStaticType(), eLeft, eRight, callerReceiverLocation), this));
                    result = result.lub(innerState);
            }
        }
        //AccessChild ac = new AccessChild(callerReceiverVte.getStaticType(), getLeft(),, callerReceiverLocation);
        return result;
    }
}