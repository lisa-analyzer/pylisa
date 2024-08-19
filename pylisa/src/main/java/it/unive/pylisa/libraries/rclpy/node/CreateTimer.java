package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.libraries.rclpy.timer.ROSTimerCallback;

public class CreateTimer extends NaryExpression implements PluggableStatement {
	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	protected Statement st;

	protected CreateTimer(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	public static CreateTimer build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreateTimer(cfg, location, "create_timer", exprs);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		ROSTimerCallback callback = new ROSTimerCallback(this.getCFG(), (SourceCodeLocation) getLocation(),
				getSubExpressions()[2]);
		try {
			callback.snooping(interprocedural, state, new ExpressionSet[] { params[2] }, expressions);
		} catch (Exception e) {
		}
		return state;
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
