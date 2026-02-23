package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.cfg.type.LiSAHttpServiceEndpoint;
import it.unive.lisa.symbolic.operators.network.HttpServiceCreation;
import it.unive.lisa.symbolic.operators.network.HttpServiceEndpointCreation;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.statement.LazyEvaluatedVariadicExpression;
import it.unive.pylisa.symbolic.operators.LazyEvaluatedExpressionOperator;

import java.util.HashMap;

public class Post extends VariadicExpression implements PluggableStatement {
    protected Statement st;
    public Post(
            CFG cfg,
            CodeLocation location,
            Expression[] params) {
        super(cfg, location, "fastapi.post", params, new HashMap<>() {{
            put("path", 1);
        }});
    }

    @Override
    protected int compareSameClassAndParams(
            Statement o) {
        return 0;
    }


    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression[] combination, StatementStore<A> expressions) throws SemanticException {
        System.out.println("fastapi.post::fwdVariadicSemantics");
        it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
                .operator(HttpServiceEndpointCreation.POST)
                .varargsOperand("target", combination[0])
                .varargsOperand("path", combination[getVarArgsIndex().get("path")])
                .staticType(LiSAHttpServiceEndpoint.INSTANCE)
                .location(getLocation())
                .build();
        Expression[] params = new Expression[1];
        params[0] = new VariableRef(getCFG(), getLocation(), "callback");
        LazyEvaluatedVariadicExpression lazy = new LazyEvaluatedVariadicExpression(Untyped.INSTANCE, expr, LazyEvaluatedExpressionOperator.INSTANCE, getLocation(), params);
        //return interprocedural.getAnalysis().assign(state, new NetworkIdentifier(.INSTANCE, "FastAPI_HTTPService", false, null, getLocation()), expr, this);*/
        return interprocedural.getAnalysis().smallStepSemantics(state, lazy, this);
        // ritorna una funzione "astratta" (HttpServiceCreation) dove alcuni parametri sono già settati (path), e altri (callback) vengono presi quando viene applicata.
        // @app.post("/add")
        // def f():
        //  .....
        // desugar: app.post("/add")(f) -> in LiSA: FunctionApply(app.post("/add"), f)
        // app.post -> fastapi.Post.java semantics (questa classe) -> sss di LazyEvaluatedVariadicExpression
        //      -> NetworkDomain pusha una PartialFunction -> x = PartialFunction(HttpServiceCreation, "/add")[callback]
        // FunctionApply(PartialFunction(HttpServiceCreation, path)[callback], f) -> applica f alla "partial function"
        //          -> f viene mappato alla variableRef callback -> PartialFunction(HttpServiceCreation("/add"), [callback](f)) = Function(HttpServiceCreation("/add", callback))
        //          ->

        // 1. Function deve essere NativeCFG?
        // 2. FunctionApply forwardSemantics:
        //      if left instanceof PartialFunction pf:
        //          pf.getInnerExpression().getParameters().add(callback)
        //          smallStepSemantics(state, pf)

        // app.post("/add")(f) -> ritorna f -> devo trovare un modo per ributtare f dentro il computedExpression.
        // deve essere "configurabile" (es. metodo astratto dove posso scrivere la mia logica), o con un metodo addHook(new AfterEvaluation(....) { fwdSemantics(...) {} }
        //      WHY: generico, non deve essere modellato dentro HttpServiceCreation, perchè HttpServiceCreation viene usato da fastapi, flask, spring, ...
        //  enable analysis of:
        // @app.post("/add")
        // @app.get("/add")
        // def f():
        //  ...
        // -> app.post("/add")app.get("/add")(f)) -> app.get("/add")(f) ritorna sempre f -> app.post("/add")(f) -> f

    }

    public static Post build(
            CFG cfg,
            CodeLocation location,
            Expression[] exprs) {
        return new Post(cfg, location, exprs);
    }

    @Override
    final public void setOriginatingStatement(
            Statement st) {
        this.st = st;
    }
}
