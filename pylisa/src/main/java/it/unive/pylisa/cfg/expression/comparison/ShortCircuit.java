package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;

/**
 * Python's {@code a and b} and {@code a or b}: the left operand is evaluated
 * first; {@code and} yields it where it is false and evaluates and yields the
 * right operand where it is true, {@code or} the other way around. The right
 * operand is evaluated only where Python evaluates it, and the result is an
 * operand, not a boolean.
 */
final class ShortCircuit {

	private ShortCircuit() {
	}

	/**
	 * Computes the semantics of a short-circuit operator.
	 *
	 * @param <A>             the kind of abstract state
	 * @param <D>             the kind of abstract domain
	 * @param statement       the operator statement
	 * @param left            the left operand
	 * @param right           the right operand
	 * @param rightWhenTrue   {@code true} for {@code and} (the right operand
	 *                            is evaluated where the left one is true),
	 *                            {@code false} for {@code or}
	 * @param entryState      the state before the expression
	 * @param interprocedural the interprocedural analysis
	 * @param expressions     the store of the states of sub-expressions
	 *
	 * @return the state after the expression
	 *
	 * @throws SemanticException if the semantics cannot be computed
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Expression statement,
			Expression left,
			Expression right,
			boolean rightWhenTrue,
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		AnalysisState<A> afterLeft = left.forwardSemantics(entryState, interprocedural, expressions);
		expressions.put(left, afterLeft);
		AnalysisState<A> result = entryState.bottomExecution();
		for (SymbolicExpression value : analysis.rewrite(afterLeft, afterLeft.getExecutionExpressions(), statement)) {
			UnaryExpression negated = new UnaryExpression(value.getStaticType(), value, LogicalNegation.INSTANCE,
					value.getCodeLocation());
			Satisfiability truth = analysis.satisfies(afterLeft, value, statement);
			boolean mayBeTrue = truth != Satisfiability.NOT_SATISFIED && truth != Satisfiability.BOTTOM;
			boolean mayBeFalse = truth != Satisfiability.SATISFIED && truth != Satisfiability.BOTTOM;
			SymbolicExpression continuing = rightWhenTrue ? value : negated;
			SymbolicExpression stopping = rightWhenTrue ? negated : value;
			if (rightWhenTrue ? mayBeTrue : mayBeFalse) {
				AnalysisState<A> afterRight = right.forwardSemantics(
						analysis.assume(afterLeft, continuing, left, right), interprocedural, expressions);
				expressions.put(right, afterRight);
				result = result.lub(afterRight);
			}
			if (rightWhenTrue ? mayBeFalse : mayBeTrue)
				result = result.lub(analysis.smallStepSemantics(
						analysis.assume(afterLeft, stopping, left, statement), value, statement));
		}
		return result;
	}
}
