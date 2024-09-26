package it.unive.pylisa.libraries.pika;

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
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;

public class CreateChannel extends NaryExpression implements PluggableStatement {

	// protected Statement st;

	protected CreateChannel(
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

	public static CreateChannel build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreateChannel(cfg, location, "create_channel", exprs);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();

		PyClassType channelClassType = PyClassType.lookup("pika.Channel");

		PyNewObj channelObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(),
				channelClassType);
		AnalysisState<A> channelAS = channelObj.forwardSemanticsAux(interprocedural,
				state, params, expressions);

		result = result.lub(channelAS);

		return result;
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		// this.st = st;
	}
}
