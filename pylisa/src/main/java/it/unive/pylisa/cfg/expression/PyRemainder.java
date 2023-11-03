package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.numeric.Remainder;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import java.util.Set;
import java.util.function.Predicate;

public class PyRemainder extends Remainder {

	public PyRemainder(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		Set<Type> rts = state.getState().getRuntimeTypesOf(left, this, state.getState());
		if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(Type::isStringType))
			// this might be a string formatting
			result = state.smallStepSemantics(new PushAny(StringType.INSTANCE, getLocation()), this);
		rts = state.getState().getRuntimeTypesOf(right, this, state.getState());
		if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(Predicate.not(Type::isStringType)))
			// this might not be a string formatting
			result = result.lub(super.fwdBinarySemantics(interprocedural, state, left, right, expressions));
		return result;
	}
}
