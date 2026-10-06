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
import it.unive.lisa.program.cfg.statement.numeric.Negation;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * Python's unary minus ({@code -x}): it calls {@code type(x).__neg__(x)}. See
 * {@link PyBinaryDispatch#dispatchUnary} for the details.
 */
public class PyNegation extends Negation {

	/**
	 * Builds the negation.
	 *
	 * @param cfg      the {@link CFG} where this operation lies
	 * @param location the location where this literal is defined
	 * @param expr     the operand of this operation
	 */
	public PyNegation(
			CFG cfg,
			CodeLocation location,
			Expression expr) {
		super(cfg, location, expr);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return PyBinaryDispatch.dispatchUnary(interprocedural, state, expressions, this, expr, "__neg__");
	}
}
