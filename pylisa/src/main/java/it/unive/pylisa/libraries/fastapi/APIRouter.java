package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSAHttpService;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.HttpServiceCreation;
import it.unive.lisa.symbolic.value.Constant;
import java.util.HashMap;

public class APIRouter extends VariadicExpression implements PluggableStatement {

	protected Statement st;

	public APIRouter(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "APIRouter", params, new HashMap<>() {
			{
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
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		APIRouter other = (APIRouter) o;
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

		int prefixIndex = getVarArgsIndex().get("prefix");
		SymbolicExpression basePath = combination.length > prefixIndex
				? combination[prefixIndex]
				: new Constant(StringType.INSTANCE, "", getLocation());

		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(HttpServiceCreation.INSTANCE)
				.varargsOperand(
						"concreteType",
						new Constant(StringType.INSTANCE, "APIRouter", getLocation()))
				.varargsOperand(
						"basePath",
						basePath)
				.varargsOperand(
						"paramDelimiter",
						new Constant(StringType.INSTANCE, "{*}", getLocation()))
				.staticType(LiSAHttpService.INSTANCE)
				.location(getLocation())
				.build();

		return interprocedural.getAnalysis().assign(
				state,
				combination[0],
				expr,
				this);
	}

	public static APIRouter build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new APIRouter(cfg, location, exprs);
	}

	@Override
	public final void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}