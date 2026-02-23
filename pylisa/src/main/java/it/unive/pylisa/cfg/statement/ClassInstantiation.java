package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.*;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;

import java.util.List;
import java.util.Set;

public class ClassInstantiation extends NaryExpression {

    private final PyClassType classType;

    public ClassInstantiation(CFG cfg, CodeLocation location, PyClassType classType, Expression[] params) {
        super(cfg, location, "$ClassInstantiation", classType, params);
        this.classType = classType;
    }

    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>>
    AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A,D> interprocedural,
                                         AnalysisState<A> state,
                                         ExpressionSet[] params,
                                         StatementStore<A> expressions) throws SemanticException {
        AnalysisState<A> result = state.bottom();
        for (SymbolicExpression identifier : params[0]) {
            Set<Type> runtimeTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, identifier, this);
            for (Type t : runtimeTypes) {
                if (t instanceof PyClassType pct) {
                    GlobalVariable variable = new GlobalVariable(Untyped.INSTANCE, "$" + pct.getUnit().getName() + "::__new__", getLocation());
                    Set<Type> types = interprocedural.getAnalysis().getRuntimeTypesOf(state, variable, this);
                    for(Type _t : types) {
                        if (_t instanceof PyFunctionType ft) {
                            CodeMember cm = ft.getUnit().getFunction();
                            Call c = null;
                            if (cm instanceof NativeCFG cfg) {
                                c = new NativeCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call", List.of(cfg), getSubExpressions());
                            } else if (cm instanceof CFG cfg) {
                                c = new CFGCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call", List.of(cfg), getSubExpressions());
                            }
                            if (c != null) {
                                InstrumentedReceiverRef syntheticVariable = new InstrumentedReceiverRef(this.getCFG(), getLocation(), false);
                                PyAssign assign = new PyAssign(getCFG(), getLocation(), syntheticVariable, c);

                                result = result.lub(assign.forwardSemantics(state, interprocedural, expressions));
                                for (SymbolicExpression e : result.getExecutionExpressions()) {
                                    Set<Type> types_result = interprocedural.getAnalysis().getRuntimeTypesOf(result, e, this);
                                    for (Type __t : types_result) {
                                        if (__t instanceof ReferenceType rt) {
                                            PythonUnitAttributeAccessRef unitAttributeAccessRef = new PythonUnitAttributeAccessRef(this.getCFG(), getLocation(), pct.getUnit(), new Global(getLocation(), pct.getUnit(), "__init__", false));
                                            if (e instanceof Identifier) {
                                                Expression[] initExpressions = new Expression[getSubExpressions().length];
                                                initExpressions[0] = syntheticVariable;
                                                for (int i = 1; i < getSubExpressions().length; i++) {
                                                    initExpressions[i] = getSubExpressions()[i];
                                                }
                                                FunctionApply a = new FunctionApply(this.getCFG(), getLocation(), unitAttributeAccessRef, initExpressions);
                                                result = result.lub(a.forwardSemantics(result, interprocedural, expressions));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return result;
    }

    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }
}