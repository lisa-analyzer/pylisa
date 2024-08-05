package it.unive.pylisa.libraries.rclpy.qos;

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
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.pylisa.cfg.expression.PyAssign;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
	protected Statement st;

	public Init(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		super(cfg, location, "__init__", exprs);
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
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public String toString() {
		return "QosProfile";
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state;
		Expression self = getSubExpressions()[0];
		UnaryExpression _avoid_ros_namespace_conventions = getNamedParameterExpr("avoid_ros_namespace_conventions");
		Expression avoid_ros_namespace_convention = _avoid_ros_namespace_conventions != null ? _avoid_ros_namespace_conventions.getSubExpression()
				: new FalseLiteral(this.getCFG(), getLocation());
		AccessInstanceGlobal aig_avoidRosNamespaceConventions = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "avoid_ros_namespace_conventions");
		PyAssign pyAssign = new PyAssign(getCFG(), getLocation(), aig_avoidRosNamespaceConventions, avoid_ros_namespace_convention);
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