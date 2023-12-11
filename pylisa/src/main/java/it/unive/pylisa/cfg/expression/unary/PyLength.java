package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.value.StringLength;

public class PyLength extends it.unive.lisa.program.cfg.statement.UnaryExpression {

	public PyLength(
			CFG cfg,
			SourceCodeLocation location,
			Expression exp) {
		super(cfg, location, "len", exp);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		if (state.getState().getRuntimeTypesOf(expr, this, state.getState()).stream().anyMatch(Type::isStringType)) {
			// String len
			return state.smallStepSemantics(
					new UnaryExpression(Int32Type.INSTANCE, expr, StringLength.INSTANCE, getLocation()), this);
		}
		return state.bottom();
	}

}