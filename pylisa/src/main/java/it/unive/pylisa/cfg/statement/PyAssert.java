package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryStatement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.pylisa.cfg.type.PyExceptionType;

/**
 * The Python statement {@code assert condition}.
 * <p>
 * Execution continues only where the condition may hold; where it may not, an
 * {@code AssertionError} is raised. An abstract state after the statement thus
 * describes exactly the executions in which the assertion passed.
 * </p>
 * <p>
 * The optional message of the assertion is not evaluated: it only affects the
 * value carried by the raised exception, and Python evaluates it only when the
 * assertion fails.
 * </p>
 */
public class PyAssert extends UnaryStatement {

	/**
	 * Builds the statement.
	 *
	 * @param cfg       the CFG the statement belongs to
	 * @param location  the location of the statement in the source
	 * @param condition the asserted condition
	 */
	public PyAssert(
			CFG cfg,
			CodeLocation location,
			Expression condition) {
		super(cfg, location, "assert", condition);
	}

	/**
	 * Yields the asserted condition.
	 *
	 * @return the condition
	 */
	public Expression getCondition() {
		return getSubExpression();
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression condition,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		AnalysisState<A> passed = analysis.assume(state, condition, this, this);
		UnaryExpression negated = new UnaryExpression(BoolType.INSTANCE, condition, LogicalNegation.INSTANCE,
				getLocation());
		AnalysisState<A> failed = analysis.assume(state, negated, this, this);
		// a statement has no value: the condition, which may mention
		// temporaries of the enclosing function, is not left behind, neither
		// on the normal path nor on the error one
		Skip none = new Skip(getLocation());
		AnalysisState<A> raised = analysis.moveExecutionToError(analysis.smallStepSemantics(failed, none, this),
				new AnalysisState.Error(PyExceptionType.ASSERTION_ERROR, this), this);
		return analysis.smallStepSemantics(passed, none, this).lub(raised);
	}
}
