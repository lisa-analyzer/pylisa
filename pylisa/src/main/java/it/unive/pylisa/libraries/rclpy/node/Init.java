package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyStringLiteral;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
	protected Statement st;

	public Init(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		super(cfg, location, "init", exprs);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Init build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Init(cfg, location, exprs);
	}

	@Override
	public String toString() {
		return "Node";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();

		Expression self = getSubExpressions()[0];
		Expression node_name = getSubExpressions()[1] instanceof NamedParameterExpression
				? getNamedParameterExpr("node_name").getSubExpression()
				: getSubExpressions()[1];
		UnaryExpression _namespace = getNamedParameterExpr("namespace");

		UnaryExpression _start_parameter_services = getNamedParameterExpr("start_parameter_services");
		Expression start_parameter_services = _start_parameter_services != null
				? _start_parameter_services.getSubExpression()
				: new TrueLiteral(this.getCFG(), getLocation());

		UnaryExpression _enable_rosout = getNamedParameterExpr("enable_rosout");
		Expression enable_rosout = _enable_rosout != null ? _enable_rosout.getSubExpression()
				: new TrueLiteral(this.getCFG(), getLocation());

		Expression namespace = _namespace != null ? _namespace.getSubExpression()
				: new PyStringLiteral(this.getCFG(), getLocation(), "", "\"");

		AccessInstanceGlobal nodeName = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "node_name");
		PyAssign pyAssign = new PyAssign(getCFG(), getLocation(), nodeName, node_name);
		result = result.lub(pyAssign.forwardSemantics(state, interprocedural, expressions));

		AccessInstanceGlobal namespaceAIG = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "namespace");
		pyAssign = new PyAssign(getCFG(), getLocation(), namespaceAIG, namespace);
		result = result.lub(pyAssign.forwardSemantics(state, interprocedural, expressions));

		AccessInstanceGlobal startParamSvc = new AccessInstanceGlobal(st.getCFG(), getLocation(), self,
				"start_parameter_services");
		pyAssign = new PyAssign(getCFG(), getLocation(), startParamSvc, start_parameter_services);
		result = result.lub(pyAssign.forwardSemantics(state, interprocedural, expressions));

		AccessInstanceGlobal enableRosOut = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "enable_rosout");
		pyAssign = new PyAssign(getCFG(), getLocation(), enableRosOut, enable_rosout);
		result = result.lub(pyAssign.forwardSemantics(state, interprocedural, expressions));

		return result;
	}

	public NamedParameterExpression getNamedParameterExpr(
			String name) {
		for (Expression e : getSubExpressions()) {
			if (e instanceof NamedParameterExpression
					&& ((NamedParameterExpression) e).getParameterName().equals(name)) {
				return ((NamedParameterExpression) e);
			}
		}
		return null;
	}
}