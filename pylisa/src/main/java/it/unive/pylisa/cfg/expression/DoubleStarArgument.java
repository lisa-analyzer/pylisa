package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * A double-starred argument of a call ({@code f(**d)}): its value is the
 * mapping, whose entries are passed as keyword arguments when the arguments are
 * assigned to the parameters of the callee (see {@code PyAssigningStrategy}).
 */
public class DoubleStarArgument extends UnaryExpression {

	/**
	 * Builds the argument.
	 *
	 * @param cfg  the cfg where the argument is
	 * @param loc  the location of the argument
	 * @param expr the mapping
	 */
	public DoubleStarArgument(
			CFG cfg,
			CodeLocation loc,
			Expression expr) {
		super(cfg, loc, "**", expr);
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
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		ScopeId<A> context = interprocedural.getCurrentContext();
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, context, this);
	}
}
