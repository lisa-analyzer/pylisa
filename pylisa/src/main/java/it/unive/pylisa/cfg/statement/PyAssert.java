package it.unive.pylisa.cfg.statement;

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
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Set;

/**
 * The {@code assert condition} statement (also {@code assert condition, msg}).
 * If the condition is false (as in python, it can be a value of any type, see
 * {@link PyNot}), the statement raises {@code AssertionError}; otherwise, the
 * execution continues. As in JLiSA, the message is always evaluated, and then
 * discarded.
 */
public class PyAssert extends NaryStatement {

	/**
	 * Builds the statement.
	 *
	 * @param cfg       the cfg where the statement is
	 * @param location  the location of the statement
	 * @param condition the asserted condition
	 * @param message   the message of the assertion, or {@code null}
	 */
	public PyAssert(
			CFG cfg,
			CodeLocation location,
			Expression condition,
			Expression message) {
		super(cfg, location, "assert",
				message == null ? new Expression[] { condition } : new Expression[] { condition, message });
	}

	/**
	 * Yields the asserted condition.
	 *
	 * @return the condition
	 */
	public Expression getCondition() {
		return getSubExpressions()[0];
	}

	/**
	 * Yields the message of the assertion.
	 *
	 * @return the message, or {@code null} if the assertion has none
	 */
	public Expression getMessage() {
		Expression[] subs = getSubExpressions();
		return subs.length > 1 ? subs[1] : null;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		Expression message = getMessage();
		return "assert " + getCondition() + (message != null ? ", " + message : "");
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		// the values of the sub-expressions are consumed here
		AnalysisState<A> consumed = state;
		for (Expression sub : getSubExpressions())
			consumed = consumed.forgetIdentifiers(sub.getMetaVariables(), this);

		boolean holds = false, fails = false;
		for (SymbolicExpression condition : params[0])
			for (SymbolicExpression cond : analysis.rewrite(state, condition, context, this)) {
				Satisfiability sat = analysis.satisfies(state, cond, context, this);
				if (sat == Satisfiability.BOTTOM)
					continue;
				if (sat == Satisfiability.UNKNOWN) {
					// e.g. an object, that is true unless its class defines
					// __bool__ or __len__
					Set<Type> types = analysis.getRuntimeTypesOf(state, cond, context, this);
					if (!types.isEmpty() && types.stream().allMatch(PyNot::alwaysTrue))
						sat = Satisfiability.SATISFIED;
				}
				holds |= sat != Satisfiability.NOT_SATISFIED;
				fails |= sat != Satisfiability.SATISFIED;
			}

		AnalysisState<A> result = state.bottomExecution();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		if (holds)
			result = result.lub(analysis.smallStepSemantics(consumed, new Skip(getLocation()), context, this));
		if (fails)
			raising.raise(consumed, getCFG(), getLocation(), this, LibrarySpecificationProvider.ASSERTION_ERROR);
		return raising.moveFrom(result);
	}
}
