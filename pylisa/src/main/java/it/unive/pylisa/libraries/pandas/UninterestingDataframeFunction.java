package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;

public class UninterestingDataframeFunction extends UnaryExpression implements PluggableStatement {

	protected Statement st;

	public UninterestingDataframeFunction(
			CFG cfg,
			CodeLocation location,
			Expression dataframe) {
		super(cfg, location, "uninteresting-func", dataframe);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static UninterestingDataframeFunction build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new UninterestingDataframeFunction(cfg, location, exprs[0]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		// we just return the same dataframe
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, st);
	}
}