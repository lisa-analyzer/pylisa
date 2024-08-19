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
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.rclpy.client.ROSServiceCallback;
import java.util.Arrays;

public class CreateService extends NaryExpression implements PluggableStatement {

	protected Statement st;

	protected CreateService(
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

	public static CreateService build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreateService(cfg, location, "create_service", exprs);
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
		String messageType = params[1].iterator().next().toString();
		Constant c = new Constant(StringType.INSTANCE, messageType, getLocation());
		params[1] = new ExpressionSet(c);

		PyClassType serviceClassType = PyClassType.lookup(LibrarySpecificationProvider.RCLPY_SERVICE);
		ROSServiceCallback callback = new ROSServiceCallback(this.getCFG(), (SourceCodeLocation) getLocation(),
				getSubExpressions()[3]);
		try {
			callback.snooping(interprocedural, state, new ExpressionSet[] { params[3] }, expressions);
		} catch (Exception e) {
		}
		PyNewObj serviceObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(), "__init__",
				serviceClassType, Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
		AnalysisState<A> newServiceAS = serviceObj.forwardSemanticsAux(interprocedural,
				state, Arrays.copyOfRange(params, 1, params.length), expressions);

		return result.lub(newServiceAS);
	}

	@Override
	public void setOriginatingStatement(
			Statement statement) {
		this.st = statement;
	}
}
