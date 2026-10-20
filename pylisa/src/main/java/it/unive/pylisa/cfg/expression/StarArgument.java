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
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * A starred argument of a call ({@code f(*xs)}): its value is the iterable,
 * whose elements are passed as positional arguments when the arguments are
 * assigned to the parameters of the callee (see {@code PyAssigningStrategy}).
 */
public class StarArgument extends StarExpression {

	/**
	 * Builds the argument.
	 *
	 * @param cfg  the cfg where the argument is
	 * @param loc  the location of the argument
	 * @param expr the iterable
	 */
	public StarArgument(
			CFG cfg,
			CodeLocation loc,
			Expression expr) {
		super(cfg, loc, expr);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}
}
