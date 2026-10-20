package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * The element bound to the variable of a for loop ({@code for x in iterable}):
 * any of the elements of the iterable (see {@link Iteration}). Iterating never
 * raises {@code IndexError}, as the loop stops when the elements are over.
 */
public class PyIteratedElement extends UnaryExpression {

	/**
	 * Builds the expression.
	 *
	 * @param cfg      the cfg where the expression is
	 * @param location the location of the expression
	 * @param iterable the iterable
	 */
	public PyIteratedElement(
			CFG cfg,
			CodeLocation location,
			Expression iterable) {
		super(cfg, location, "element", iterable);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		return "element(" + getSubExpression() + ")";
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return Iteration.elements(interprocedural.getAnalysis(), state, getSubExpression(), expr, this);
	}
}
