package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.string.StringConstant;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyModuleType;

import java.util.Set;

public class AttributeAccess extends UnaryExpression {
    String target;
    public AttributeAccess(CFG cfg, CodeLocation location, Expression access, String target) {
        super(cfg, location, "$AttributeAccess", access);
        this.target = target;
    }

    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression expr, StatementStore<A> expressions) throws SemanticException {
        Set<Type> runTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, expr, this);
        AnalysisState<A> result = state.bottom();
        for (Type t : runTypes) {
            if (t instanceof ReferenceType rt) {
                t = rt.getInnerType();
            }
            if (t instanceof PyModuleType mt) {
                // t is a module type, then I need to access the global.
                StringConstant m =  state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class).getState(mt.getUnit().getName());
                if (m.isBottom()) {
                    return state.bottom(); // the module is not allocated.
                }
                GlobalVariable access = new GlobalVariable(
                        Untyped.INSTANCE,
                        m.value + "::" + target,
                        getLocation());
                result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state, access, this));
                continue;
            }
            if (t instanceof PyClassType ct) {
                // t is a module type, then I need to access the global.
                StringConstant m =  state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class).getState(ct.getUnit().getName());
                if (m.isBottom()) {
                    return state.bottom(); // the module is not allocated.
                }
                GlobalVariable access = new GlobalVariable(
                        Untyped.INSTANCE,
                        m.value + "::" + target,
                        getLocation());
                result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state, access, this));
                continue;
            }
        }

        return result;
    }

    @Override
    public String toString() {
        return getSubExpression() + "::" + target;
    }
    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }

}
