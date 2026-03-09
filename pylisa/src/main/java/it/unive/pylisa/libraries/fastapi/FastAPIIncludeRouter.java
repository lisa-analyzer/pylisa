package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkActiveNode;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.ActiveNodeCompose;
import java.util.HashMap;

public class FastAPIIncludeRouter extends VariadicExpression implements PluggableStatement {

	protected Statement st;

	public FastAPIIncludeRouter(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "include_router", params, new HashMap<>() {
			{
				put("router", 1);
				put("prefix", 2);
				put("tags", 3);
				put("dependencies", 4);
				put("responses", 5);
				put("deprecated", 6);
				put("include_in_schema", 7);
				put("default_response_class", 8);
				put("callbacks", 9);
				put("generate_unique_id_function", 10);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		FastAPIIncludeRouter other = (FastAPIIncludeRouter) o;
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

		System.out.println("FastApiIncludeRouter::fwdVariadicSemantics");

		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ActiveNodeCompose.INSTANCE)
				.varargsOperand("target", combination[0])
				.varargsOperand(
						"child",
						combination[getVarArgsIndex().get("router")])
				.varargsOperand(
						"prefix",
						combination[getVarArgsIndex().get("prefix")])
				.staticType(LiSANetworkActiveNode.INSTANCE)
				.location(getLocation())
				.build();
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}

	public static FastAPIIncludeRouter build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new FastAPIIncludeRouter(cfg, location, exprs);
	}

	@Override
	public final void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
