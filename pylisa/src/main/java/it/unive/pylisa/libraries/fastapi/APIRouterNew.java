package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSAHttpService;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariadicExpression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.HttpServiceCreation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;

import java.util.HashMap;

public class APIRouterNew extends VariadicExpression implements PluggableStatement {
    protected Statement st;
    public APIRouterNew(
            CFG cfg,
            CodeLocation location,
            Expression[] params) {
        super(cfg, location, "APIRouterNew", params, new HashMap<>() {{
            put("prefix", 1);
            put("tags", 2);
            put("dependencies", 3);
            put("default_response_class", 4);
            put("responses", 5);
            put("callbacks", 6);
            put("routes", 7);
            put("redirect_slashes", 8);
            put("default", 9);
            put("dependency_overrides_provider", 10);
            put("route_class", 11);
            put("on_startup", 12);
            put("on_shutdown", 13);
            put("lifespan", 14);
            put("deprecated", 15);
            put("include_in_schema", 16);
            put("generate_unique_id_function", 17);
        }});
    }

    @Override
    protected int compareSameClassAndParams(
            Statement o) {
        return 0;
    }


    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression[] combination, StatementStore<A> expressions) throws SemanticException {
        System.out.println("FastAPINew::fwdVariadicSemantics");
        PyNewObj newObj = new PyNewObj(this.getCFG(), getLocation(), "__init__", PyClassType.lookup("fastapi.APIRouter"), getSubExpressions());
        return newObj.forwardSemantics(state, interprocedural, expressions);
    }

    public static APIRouterNew build(
            CFG cfg,
            CodeLocation location,
            Expression[] exprs) {
        return new APIRouterNew(cfg, location, exprs);
    }

    @Override
    final public void setOriginatingStatement(
            Statement st) {
        this.st = st;
    }
}