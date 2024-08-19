package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.ros.lisa.symbolic.operators.ros.ROSTopicNameExpansion;
import java.util.HashSet;
import java.util.Set;

public class SemanticsHelpers {

	public static <A extends AbstractState<A>> ExpressionSet nameExpansion(
			Statement st,
			Expression node,
			ExpressionSet exprs,
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			StatementStore<A> expressions)
			throws SemanticException {

		Set<SymbolicExpression> exprSet = new HashSet<>();
		AccessInstanceGlobal aigNS = new AccessInstanceGlobal(st.getCFG(), st.getLocation(), node,
				"namespace");
		// compute semantics
		AnalysisState<A> aigSemanticsNS = aigNS.forwardSemantics(state, interprocedural, expressions);

		// [AIG] self.node_name
		AccessInstanceGlobal aigNodeName = new AccessInstanceGlobal(st.getCFG(), st.getLocation(), node,
				"node_name");
		// compute semantics
		AnalysisState<
				A> aigSemanticsNodeName = aigNodeName.forwardSemantics(aigSemanticsNS, interprocedural, expressions);

		for (SymbolicExpression s : exprs) {
			for (SymbolicExpression eNS : aigSemanticsNS.getComputedExpressions()) {
				ExpressionSet vesNS = state.getState().rewrite(eNS, st, state.getState());
				for (SymbolicExpression eNodeName : aigSemanticsNodeName.getComputedExpressions()) {
					ExpressionSet vesNodeName = state.getState().rewrite(eNodeName, st, state.getState());
					for (SymbolicExpression veNS : vesNS) {
						for (SymbolicExpression veNodeName : vesNodeName)
							exprSet.add(new TernaryExpression(StringType.INSTANCE, s, veNS, veNodeName,
									new ROSTopicNameExpansion(), st.getLocation()));
					}
				}
			}
		}
		return new ExpressionSet(exprSet);
	}

	public static NamedParameterExpression getNamedParameterExpr(
			Expression[] subExpressions,
			String name) {
		for (Expression e : subExpressions) {
			if (e instanceof NamedParameterExpression
					&& ((NamedParameterExpression) e).getParameterName().equals(name)) {
				return ((NamedParameterExpression) e);
			}
		}
		return null;
	}

	public static String getMessageType() {
		return "ciao";
	}

}
