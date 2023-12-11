package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyStringLiteral;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
	protected Statement st;

	public Init(CFG cfg, CodeLocation location, Expression[] exprs) {
		super(cfg, location, "init", exprs);
	}

	public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Init(cfg, location, exprs);
	}

	@Override
	public String toString() {
		return "Node";
	}

	@Override
	public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = state;
		Expression self = getSubExpressions()[0];
		Expression node_name = getSubExpressions()[1] instanceof NamedParameterExpression
				? getNamedParameterExpr("node_name").getSubExpression()
				: getSubExpressions()[1];
		UnaryExpression _namespace = getNamedParameterExpr("namespace");

		Expression namespace = _namespace != null ? _namespace.getSubExpression()
				: new PyStringLiteral(this.getCFG(), getLocation(), "", "\"");

		AccessInstanceGlobal nodeName = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "node_name");
		PyAssign pyAssign = new PyAssign(getCFG(), getLocation(), nodeName, node_name);
		result = result.lub(pyAssign.semantics(result, interprocedural, expressions));

		namespace.semantics(state, interprocedural, expressions).getComputedExpressions();
		AccessInstanceGlobal namespaceAIG = new AccessInstanceGlobal(st.getCFG(), getLocation(), self, "namespace");
		pyAssign = new PyAssign(getCFG(), getLocation(), namespaceAIG, namespace);
		result = result.lub(pyAssign.semantics(result, interprocedural, expressions));

		return result;
	}

	public NamedParameterExpression getNamedParameterExpr(String name) {
		for (Expression e : getSubExpressions()) {
			if (e instanceof NamedParameterExpression
					&& ((NamedParameterExpression) e).getParameterName().equals(name)) {
				return ((NamedParameterExpression) e);
			}
		}
		return null;
	}
}