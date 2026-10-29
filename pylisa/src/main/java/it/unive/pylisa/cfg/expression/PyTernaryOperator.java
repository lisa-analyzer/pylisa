package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
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

	/**
	 * Overridden (rather than relying on {@link #forwardSemanticsAux}) since
	 * the default {@code NaryExpression.forwardSemantics} evaluates all the
	 * sub-expressions before {@code forwardSemanticsAux} is invoked: here, only
	 * the condition is always evaluated, and then only the branches that the
	 * condition can select, each assuming the condition (or its negation).
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Expression[] sub = getSubExpressions();
		Expression condition = sub[0];
		Expression ifTrue = sub[1];
		Expression ifFalse = sub[2];

		AnalysisState<A> postCondition = condition.forwardSemantics(entryState, interprocedural, expressions);
		expressions.put(condition, postCondition);

		AnalysisState<A> trueState = postCondition.bottomExecution();
		AnalysisState<A> falseState = postCondition.bottomExecution();
		boolean canBeTrue = false, canBeFalse = false;
		for (SymbolicExpression cond : analysis.rewrite(postCondition, postCondition.getExecutionExpressions(),
				context, this)) {
			Satisfiability sat = analysis.satisfies(postCondition, cond, context, this);
			if (sat == Satisfiability.BOTTOM)
				continue;
			if (sat != Satisfiability.NOT_SATISFIED) {
				canBeTrue = true;
				trueState = trueState.lub(analysis.assume(postCondition, cond, context, condition, ifTrue));
			}
			if (sat != Satisfiability.SATISFIED) {
				canBeFalse = true;
				UnaryExpression negated = new UnaryExpression(cond.getStaticType(), cond, LogicalNegation.INSTANCE,
						cond.getCodeLocation());
				falseState = falseState.lub(analysis.assume(postCondition, negated, context, condition, ifFalse));
			}
		}

		// the value of the condition is consumed here; the errors raised by
		// the condition are kept
		AnalysisState<A> result = postCondition.bottomExecution()
				.forgetIdentifiers(condition.getMetaVariables(), this);
		if (canBeTrue)
			result = result.lub(branch(ifTrue, trueState.forgetIdentifiers(condition.getMetaVariables(), this),
					interprocedural, expressions));
		if (canBeFalse)
			result = result.lub(branch(ifFalse, falseState.forgetIdentifiers(condition.getMetaVariables(), this),
					interprocedural, expressions));
		return result;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> branch(
			Expression branch,
			AnalysisState<A> state,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = branch.forwardSemantics(state, interprocedural, expressions);
		expressions.put(branch, result);
		if (result.getExecutionState().isBottom())
			// the branch never completes normally (e.g., it raises): only its
			// errors contribute, and not the expressions left on its stack
			return result.withExecution(result.getExecution().bottom());
		// the value of the branch is the value of this expression: its meta
		// variables (e.g., the return value of a call) are forgotten by the
		// statement that consumes it
		getMetaVariables().addAll(branch.getMetaVariables());
		return result;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// never invoked: forwardSemantics evaluates the sub-expressions
		throw new SemanticException("Conditional expressions are evaluated by forwardSemantics");
	}
}
