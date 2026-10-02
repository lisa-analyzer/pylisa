package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.lattices.Satisfiability;
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Expression[] sub = getSubExpressions();
		Expression condition = sub[0];
		Expression ifTrue = sub[1];
		Expression ifFalse = sub[2];

		AnalysisState<A> postCondition = condition.forwardSemantics(entryState, interprocedural, expressions);
		// Python evaluates only the branch the condition selects: each branch
		// is evaluated where the condition may select it, and the outcomes are
		// joined
		AnalysisState<A> result = entryState.bottomExecution();
		for (SymbolicExpression cond : interprocedural.getAnalysis().rewrite(
				postCondition,
				postCondition.getExecution().getComputedExpressions(),
				this)) {
			UnaryExpression negated = new UnaryExpression(
					cond.getStaticType(),
					cond,
					LogicalNegation.INSTANCE,
					cond.getCodeLocation());
			Satisfiability selected = interprocedural.getAnalysis().satisfies(postCondition, cond, this);
			if (selected != Satisfiability.NOT_SATISFIED && selected != Satisfiability.BOTTOM)
				result = result.lub(ifTrue.forwardSemantics(
						interprocedural.getAnalysis().assume(postCondition, cond, condition, ifTrue),
						interprocedural,
						expressions));
			if (selected != Satisfiability.SATISFIED && selected != Satisfiability.BOTTOM)
				result = result.lub(ifFalse.forwardSemantics(
						interprocedural.getAnalysis().assume(postCondition, negated, condition, ifFalse),
						interprocedural,
						expressions));
		}
		return result;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// this should be unreachable
		throw new SemanticException("Auxiliary semantics should be unreachable");
	}
}
