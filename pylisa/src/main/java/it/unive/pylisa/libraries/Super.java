package it.unive.pylisa.libraries;

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
import it.unive.lisa.symbolic.value.operator.binary.TypeCast;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import it.unive.lisa.type.*;
import it.unive.pylisa.cfg.type.PyClassType;

import java.util.HashSet;


public class Super extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
    protected Statement st;

    protected Super(CFG cfg, CodeLocation location, String constructName,
                    Expression[] expressions) {
        super(cfg, location, constructName, expressions);
    }

    public static Super build(CFG cfg, CodeLocation location, Expression[] exprs) {
        return new Super (cfg, location, "super", exprs);
    }

    @Override
    public String toString() {
        return "super";
    }

    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state, ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions) throws SemanticException {
        AnalysisState<A, H, V, T> result = state.bottom();
        if (st.getCFG().getDescriptor().isInstance()) {
            CodeLocation codeLocation = st.getCFG().getDescriptor().getVariables().get(0).getLocation();
            // first formal is always (?) the pointer to the Object
            Parameter self = st.getCFG().getDescriptor().getFormals()[0];
            ReferenceType refTypeFrom = (ReferenceType) self.getStaticType();
            PyClassType classTypeFrom = (PyClassType) refTypeFrom.getInnerType();
            CompilationUnit superObj = (CompilationUnit)classTypeFrom.getUnit().getImmediateAncestors().toArray()[0];
            PyClassType classTypeTo = PyClassType.lookup(superObj.getName());
            ReferenceType refTypeTo = new ReferenceType(classTypeTo);
            HashSet<Type> tokenTypes = new HashSet<>();
            tokenTypes.add(refTypeTo);
            TypeTokenType tokenTypeTo = new TypeTokenType(tokenTypes);
            result = result.lub(state.smallStepSemantics(
                    new it.unive.lisa.symbolic.value.BinaryExpression(
                            refTypeTo,
                            new Constant(classTypeFrom.getReference(), refTypeFrom, codeLocation),
                            new Constant(tokenTypeTo, tokenTypes, codeLocation),
                            TypeCast.INSTANCE,
                            codeLocation),
                    this));
        }
        return result;
    }

    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
}