package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;

public class PyTernaryOperator extends NaryExpression {

	public PyTernaryOperator(
			CFG cfg,
			CodeLocation location,
			Expression condition,
			Expression ifTrue,
			Expression ifFalse) {
		super(cfg, location, "?", ifTrue.getStaticType().commonSupertype(ifFalse.getStaticType()), condition, ifTrue,
				ifFalse);
	}

	@Override
	public String toString() {
		Expression[] sub = getSubExpressions();
		return sub[1] + " if " + sub[0] + " else " + sub[2];
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Expression[] sub = getSubExpressions();
		Expression condition = sub[0];
		Expression ifTrue = sub[1];
		Expression ifFalse = sub[2];

		AnalysisState<A> postCondition = condition.forwardSemantics(state, interprocedural, expressions);
		for (SymbolicExpression cond : analysis.rewrite(state,
				postCondition.getExecutionExpressions(),
				this)) {
			UnaryExpression negated = new UnaryExpression(
					cond.getStaticType(),
					cond,
					LogicalNegation.INSTANCE,
					cond.getCodeLocation());

			switch (analysis.satisfies(postCondition, cond, this)) {
			case BOTTOM:
				return state.bottom();
			case NOT_SATISFIED:
				return ifFalse.forwardSemantics(
						analysis.assume(postCondition, cond, condition, ifTrue),
						interprocedural,
						expressions);
			case SATISFIED:
				return ifTrue.forwardSemantics(
						analysis.assume(postCondition, negated, condition, ifFalse),
						interprocedural,
						expressions);
			case UNKNOWN:
				return ifTrue
						.forwardSemantics(
								analysis.assume(postCondition, cond, condition, ifTrue),
								interprocedural,
								expressions)
						.lub(ifFalse.forwardSemantics(
								analysis.assume(postCondition, negated, condition, ifFalse),
								interprocedural,
								expressions));
			}
		}

		return state.top();
	}
}
