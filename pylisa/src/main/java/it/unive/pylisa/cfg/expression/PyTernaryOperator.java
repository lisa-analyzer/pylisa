package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(AnalysisState<A> entryState, InterproceduralAnalysis<A, D> interprocedural, StatementStore<A> expressions) throws SemanticException {
		Expression[] sub = getSubExpressions();
		Expression condition = sub[0];
		Expression ifTrue = sub[1];
		Expression ifFalse = sub[2];

		AnalysisState<A> postCondition = condition.forwardSemantics(entryState, interprocedural, expressions);
		for (SymbolicExpression cond : interprocedural.getAnalysis().rewrite(
				entryState,
				postCondition.getExecution().getComputedExpressions(),
				this)) {
			UnaryExpression negated = new UnaryExpression(
					cond.getStaticType(),
					cond,
					LogicalNegation.INSTANCE,
					cond.getCodeLocation());

			switch (interprocedural.getAnalysis().satisfies(postCondition, cond, this)) {
				case BOTTOM:
					return entryState.bottom();
				case NOT_SATISFIED:
					return ifFalse.forwardSemantics(
							interprocedural.getAnalysis().assume(postCondition, cond, condition, ifTrue),
							interprocedural,
							expressions);
				case SATISFIED:
					return ifTrue.forwardSemantics(
							interprocedural.getAnalysis().assume(postCondition, negated, condition, ifFalse),
							interprocedural,
							expressions);
				case UNKNOWN:
					return ifTrue
							.forwardSemantics(
									interprocedural.getAnalysis().assume(postCondition, cond, condition, ifTrue),
									interprocedural,
									expressions)
							.lub(ifFalse.forwardSemantics(
									interprocedural.getAnalysis().assume(postCondition, negated, condition, ifFalse),
									interprocedural,
									expressions));
			}
		}

		return entryState.top();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
		// this should be unreachable
		throw new SemanticException("Auxiliary semantics should be unreachable");
	}
}
