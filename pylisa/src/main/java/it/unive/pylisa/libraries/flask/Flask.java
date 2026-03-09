package it.unive.pylisa.libraries.flask;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkActiveNode;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.ActiveNodeCreation;
import it.unive.lisa.symbolic.value.Constant;
import java.util.HashMap;

public class Flask extends VariadicExpression implements PluggableStatement {

	protected Statement st;

	public Flask(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "Flask", params, new HashMap<>() {
			{
				put("import_name", 1);
				put("static_url_path", 2);
				put("static_folder", 3);
				put("static_host", 4);
				put("host_matching", 5);
				put("subdomain_matching", 6);
				put("template_folder", 7);
				put("instance_path", 8);
				put("instance_relative_config", 9);
				put("root_path", 10);
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

		System.out.println("FlaskInit::fwdVariadicSemantics");

		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ActiveNodeCreation.INSTANCE)
				.varargsOperand(
						"concreteType",
						new Constant(StringType.INSTANCE, "Flask", getLocation()))
				.varargsOperand(
						"basePath",
						new Constant(StringType.INSTANCE, "", getLocation()))
				.varargsOperand(
						"paramDelimiter",
						new Constant(StringType.INSTANCE, "<*>", getLocation()))
				.staticType(LiSANetworkActiveNode.INSTANCE)
				.location(getLocation())
				.build();

		return interprocedural.getAnalysis().assign(
				state,
				combination[0],
				expr,
				this);
	}

	public static Flask build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Flask(cfg, location, exprs);
	}

	@Override
	public final void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}