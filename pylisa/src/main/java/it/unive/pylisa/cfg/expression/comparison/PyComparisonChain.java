package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.Untyped;
import java.util.Arrays;

/**
 * A chain of comparisons such as {@code a < b <= c}, which Python evaluates as
 * {@code a < b and b <= c} with each operand evaluated at most once: the
 * operands are evaluated from the left, each one after the previous
 * comparison held, and the chain is {@code False} at the first comparison that
 * does not hold. The value of every inner operand is kept in a temporary
 * variable, so that it is not evaluated again.
 */
public class PyComparisonChain extends NaryExpression {

	private final BinaryOperator[] operators;

	/**
	 * Builds the chain.
	 *
	 * @param cfg       the CFG the expression belongs to
	 * @param location  the location of the expression
	 * @param operands  the operands, at least three
	 * @param operators the comparison operators, one fewer than the operands
	 */
	public PyComparisonChain(
			CFG cfg,
			CodeLocation location,
			Expression[] operands,
			BinaryOperator[] operators) {
		super(cfg, location, "chain", BoolType.INSTANCE, operands);
		if (operators.length != operands.length - 1)
			throw new IllegalArgumentException("A chain of " + operands.length + " operands needs "
					+ (operands.length - 1) + " operators");
		this.operators = operators.clone();
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return Arrays.toString(operators).compareTo(Arrays.toString(((PyComparisonChain) o).operators));
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Expression first = getSubExpressions()[0];
		AnalysisState<A> afterFirst = first.forwardSemantics(entryState, interprocedural, expressions);
		expressions.put(first, afterFirst);
		AnalysisState<A> result = entryState.bottomExecution();
		for (SymbolicExpression value : interprocedural.getAnalysis().rewrite(afterFirst,
				afterFirst.getExecutionExpressions(), this))
			result = result.lub(link(0, value, afterFirst, interprocedural, expressions));
		return result;
	}

	/**
	 * Evaluates the comparison between an operand, whose value is given, and
	 * the next one, and the rest of the chain where the comparison holds.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> link(
			int index,
			SymbolicExpression left,
			AnalysisState<A> state,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Expression next = getSubExpressions()[index + 1];
		AnalysisState<A> afterNext = next.forwardSemantics(state, interprocedural, expressions);
		expressions.put(next, afterNext);
		boolean last = index == operators.length - 1;
		AnalysisState<A> result = state.bottomExecution();
		ExpressionSet values = analysis.rewrite(afterNext, afterNext.getExecutionExpressions(), this);
		for (SymbolicExpression value : values) {
			// the value of an inner operand is kept, so that the next
			// comparison does not evaluate it again
			SymbolicExpression right = value;
			AnalysisState<A> kept = afterNext;
			if (!last) {
				right = new Variable(Untyped.INSTANCE, "$chain" + index + "@" + getLocation(), getLocation());
				kept = analysis.assign(afterNext, (Variable) right, value, this);
			}
			SymbolicExpression comparison = new BinaryExpression(BoolType.INSTANCE, left, right, operators[index],
					getLocation());
			UnaryExpression negated = new UnaryExpression(BoolType.INSTANCE, comparison, LogicalNegation.INSTANCE,
					getLocation());
			Satisfiability holds = analysis.satisfies(kept, comparison, this);
			if (holds != Satisfiability.SATISFIED && holds != Satisfiability.BOTTOM)
				result = result.lub(analysis.smallStepSemantics(analysis.assume(kept, negated, this, this),
						new Constant(BoolType.INSTANCE, false, getLocation()), this));
			if (holds != Satisfiability.NOT_SATISFIED && holds != Satisfiability.BOTTOM) {
				AnalysisState<A> holding = analysis.assume(kept, comparison, this, this);
				result = result.lub(last
						? analysis.smallStepSemantics(holding, comparison, this)
						: link(index + 1, right, holding, interprocedural, expressions));
			}
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
		// never used: the operands are evaluated by forwardSemantics, lazily
		throw new UnsupportedOperationException("A comparison chain evaluates its operands itself");
	}
}
