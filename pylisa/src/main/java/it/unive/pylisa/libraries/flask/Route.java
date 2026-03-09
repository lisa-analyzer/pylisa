package it.unive.pylisa.libraries.flask;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkResource;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.ChannelEndpointCreation;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.statement.LazyEvaluatedVariadicExpression;
import it.unive.pylisa.symbolic.operators.LazyEvaluatedExpressionOperator;
import java.util.HashMap;

public class Route extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public Route(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "flask.router", params, new HashMap<>() {
			{
				put("path", 1);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression[] combination,
			StatementStore<A> expressions)
			throws SemanticException {
		System.out.println("flask.route::fwdVariadicSemantics");
		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ChannelEndpointCreation.HTTP_GET)
				.varargsOperand("target", combination[0])
				.varargsOperand("path", combination[getVarArgsIndex().get("path")])
				.staticType(LiSANetworkResource.INSTANCE)
				.location(getLocation())
				.build();
		Expression[] params = new Expression[1];
		params[0] = new VariableRef(getCFG(), getLocation(), "callback");
		LazyEvaluatedVariadicExpression lazy = new LazyEvaluatedVariadicExpression(Untyped.INSTANCE, expr,
				LazyEvaluatedExpressionOperator.INSTANCE, getLocation(), params);
		// return interprocedural.getAnalysis().assign(state, new
		// NetworkIdentifier(.INSTANCE, "FastAPI_HTTPService", false, null,
		// getLocation()), expr, this);*/
		return interprocedural.getAnalysis().smallStepSemantics(state, lazy, this);
	}

	public static Route build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Route(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
