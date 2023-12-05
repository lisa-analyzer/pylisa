package it.unive.pylisa.libraries.stdlib.object;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;

public class ObjectBool extends UnaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectBool(
			CFG cfg,
			CodeLocation location,
			Expression arg) {
		super(cfg, location, "__bool__", cfg.getDescriptor().getUnit().getProgram().getTypes().getBooleanType(), arg);
	}

	public static ObjectBool build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectBool(cfg, location, exprs[0]);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression arg,
			StatementStore<A> expressions)
			throws SemanticException {
		return state.smallStepSemantics(new PushAny(getStaticType(), getLocation()), st);
	}
}