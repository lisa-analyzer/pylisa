package it.unive.pylisa.libraries.pika;

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
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;

public class ChannelInit extends NaryExpression implements PluggableStatement {

	// private Statement st;

	protected ChannelInit(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static ChannelInit build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ChannelInit(cfg, location, "__init__", exprs);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		return state;
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		// this.st = st;
	}
}
