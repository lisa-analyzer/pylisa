package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;

public class NoOpFunction extends NaryExpression implements PluggableStatement {

	protected Statement st;

	protected NoOpFunction(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	public static NoOpFunction build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NoOpFunction(cfg, location, "noop-func", exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		return new NoOp(getCFG(), getLocation()).forwardSemantics(state, interprocedural, expressions);
	}
}