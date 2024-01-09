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

import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.rclpy.subscription.ROSSubscriptionCallback;

import java.util.Arrays;

public class CreateSubscription extends NaryExpression implements PluggableStatement {
	protected Statement st;

	protected CreateSubscription(
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

	public static CreateSubscription build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreateSubscription(cfg, location, "create_subscription", exprs);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();

		params[2] = SemanticsHelpers.nameExpansion(this, getSubExpressions()[0], params[2], interprocedural, state,
				expressions);

		PyClassType subscriptionClassType = PyClassType.lookup(LibrarySpecificationProvider.RCLPY_SUBSCRIPTION);

		PyNewObj subscriptionObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(), "__init__",
				subscriptionClassType, Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
		ROSSubscriptionCallback callback = new ROSSubscriptionCallback(this.getCFG(), (SourceCodeLocation) getLocation(), getSubExpressions()[4]);
		callback.snooping(interprocedural, state, new ExpressionSet[]{params[4]}, expressions);
		AnalysisState<A> newSubscriptionAS = subscriptionObj.forwardSemanticsAux(interprocedural,
				state, Arrays.copyOfRange(params, 1, params.length), expressions);

		return result.lub(newSubscriptionAS);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}