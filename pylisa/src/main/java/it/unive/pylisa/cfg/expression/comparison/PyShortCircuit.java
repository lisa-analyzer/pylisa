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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import java.util.Set;

/**
 * The short-circuit boolean operators {@code left and right} and
 * {@code left or right}. The left operand is evaluated first: if it decides the
 * result (i.e., it is false for {@code and}, true for {@code or}), the result
 * is the left operand itself, and the right one is not evaluated; otherwise,
 * the result is the right operand. As in python, the result is one of the
 * operands, of any type (e.g. {@code 0 or "x"} is {@code "x"}).
 */
public abstract class PyShortCircuit extends BinaryExpression {

	/**
	 * Builds the operator.
	 *
	 * @param cfg           the cfg where the operator is
	 * @param location      the location of the operator
	 * @param constructName the name of the operator
	 * @param left          the left operand
	 * @param right         the right operand
	 */
	protected PyShortCircuit(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression left,
			Expression right) {
		super(cfg, location, constructName, left, right);
	}

	/**
	 * Whether the left operand decides the result when it is true (as for
	 * {@code or}) rather than when it is false (as for {@code and}).
	 *
	 * @return whether a true left operand is the result
	 */
	protected abstract boolean decidedByTrue();

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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Expression left = getLeft();
		Expression right = getRight();

		AnalysisState<A> postLeft = left.forwardSemantics(entryState, interprocedural, expressions);
		expressions.put(left, postLeft);

		// the errors raised by the left operand are kept
		AnalysisState<A> decided = postLeft.bottomExecution();
		AnalysisState<A> other = postLeft.bottomExecution();
		boolean canDecide = false, canContinue = false;
		for (SymbolicExpression value : postLeft.getExecutionExpressions())
			for (SymbolicExpression cond : analysis.rewrite(postLeft, value, this)) {
				Satisfiability sat = analysis.satisfies(postLeft, cond, this);
				if (sat == Satisfiability.BOTTOM)
					continue;
				if (sat == Satisfiability.UNKNOWN) {
					// e.g. an object, that is true unless its class defines
					// __bool__ or __len__
					Set<Type> types = analysis.getRuntimeTypesOf(postLeft, cond, this);
					if (!types.isEmpty() && types.stream().allMatch(PyNot::alwaysTrue))
						sat = Satisfiability.SATISFIED;
				}
				// not cond, for values of any type (see LogicalNot)
				UnaryExpression negated = new UnaryExpression(BoolType.INSTANCE, cond, LogicalNot.INSTANCE,
						cond.getCodeLocation());
				boolean maybeTrue = sat != Satisfiability.NOT_SATISFIED;
				boolean maybeFalse = sat != Satisfiability.SATISFIED;
				if (decidedByTrue() ? maybeTrue : maybeFalse) {
					// the result is the left operand
					canDecide = true;
					AnalysisState<A> assumed = analysis.assume(postLeft, decidedByTrue() ? cond : negated, left, this);
					decided = decided.lub(assumed.withExecutionExpression(value));
				}
				if (decidedByTrue() ? maybeFalse : maybeTrue) {
					// the right operand is evaluated
					canContinue = true;
					other = other.lub(analysis.assume(postLeft, decidedByTrue() ? negated : cond, left, right));
				}
			}

		AnalysisState<A> result = postLeft.bottomExecution();
		if (canDecide && !decided.getExecutionState().isBottom()) {
			// the value of the left operand is the value of this expression:
			// its meta variables are forgotten by the statement that consumes
			// it
			result = result.lub(decided);
			getMetaVariables().addAll(left.getMetaVariables());
		}
		if (canContinue) {
			// the value of the left operand is consumed here
			AnalysisState<A> postRight = right.forwardSemantics(
					other.forgetIdentifiers(left.getMetaVariables(), this), interprocedural, expressions);
			expressions.put(right, postRight);
			if (postRight.getExecutionState().isBottom())
				// the right operand never completes normally (e.g., it
				// raises): only its errors contribute, and not the
				// expressions left on its stack
				result = result.lub(postRight.withExecution(postRight.getExecution().bottom()));
			else {
				result = result.lub(postRight);
				getMetaVariables().addAll(right.getMetaVariables());
			}
		}
		return result;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		// never invoked: forwardSemantics evaluates the operands
		throw new SemanticException("Boolean operators are evaluated by forwardSemantics");
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// never invoked: forwardSemantics evaluates the operands
		throw new SemanticException("Boolean operators are evaluated by forwardSemantics");
	}
}
