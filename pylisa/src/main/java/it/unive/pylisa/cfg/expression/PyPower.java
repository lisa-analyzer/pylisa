package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.Power;
import java.util.Set;

public class PyPower extends BinaryExpression {

	public PyPower(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "**", left, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type> rtsl = state.getState().getRuntimeTypesOf(left, this, state.getState());
		Set<Type> rtsr = state.getState().getRuntimeTypesOf(right, this, state.getState());
		if (rtsl != null && !rtsl.isEmpty() && rtsl.stream().anyMatch(Type::isNumericType)
				&& rtsr != null && !rtsr.isEmpty() && rtsr.stream().anyMatch(Type::isNumericType)) {
			return state.smallStepSemantics(
					new it.unive.lisa.symbolic.value.BinaryExpression(
							getStaticType(),
							left,
							right,
							Power.INSTANCE,
							getLocation()),
					this);
		}

		return state.bottom();
	}
}
