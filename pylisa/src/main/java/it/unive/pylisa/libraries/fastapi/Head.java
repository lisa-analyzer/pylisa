package it.unive.pylisa.libraries.fastapi;

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

public class Head extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public Head(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "fastapi.head", params, new HashMap<>() {
			{
				put("path", 1);
				put("response_model", 2);
				put("status_code", 3);
				put("tags", 4);
				put("dependencies", 5);
				put("summary", 6);
				put("description", 7);
				put("response_description", 8);
				put("responses", 9);
				put("deprecated", 10);
				put("operation_id", 11);
				put("response_model_include", 12);
				put("response_model_exclude", 13);
				put("response_model_by_alias", 14);
				put("response_model_exclude_unset", 15);
				put("response_model_exclude_defaults", 16);
				put("response_model_exclude_none", 17);
				put("include_in_schema", 18);
				put("response_class", 19);
				put("name", 20);
				put("callbacks", 21);
				put("openapi_extra", 22);
				put("generate_unique_id_function", 23);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		Head other = (Head) o;
		int cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression[] combination,
			StatementStore<A> expressions)
			throws SemanticException {
		System.out.println("fastapi.head::fwdVariadicSemantics");
		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ChannelEndpointCreation.HTTP_HEAD)
				.varargsOperand("target", combination[0])
				.varargsOperand("path", combination[getVarArgsIndex().get("path")])
				.staticType(LiSANetworkResource.INSTANCE)
				.location(getLocation())
				.build();
		Expression[] params = new Expression[1];
		params[0] = new VariableRef(getCFG(), getLocation(), "callback");
		LazyEvaluatedVariadicExpression lazy = new LazyEvaluatedVariadicExpression(Untyped.INSTANCE, expr,
				LazyEvaluatedExpressionOperator.INSTANCE, getLocation(), params);
		return interprocedural.getAnalysis().smallStepSemantics(state, lazy, this);
	}

	public static Head build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Head(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
