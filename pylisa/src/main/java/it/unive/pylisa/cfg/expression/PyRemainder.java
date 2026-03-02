package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		Set<Type> rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
		if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(Type::isStringType))
			// this might be a string formatting
			result = interprocedural.getAnalysis().smallStepSemantics(state,
					new PushAny(StringType.INSTANCE, getLocation()), this);
		rts = interprocedural.getAnalysis().getRuntimeTypesOf(state, right, this);
		if (rts != null && !rts.isEmpty() && rts.stream().anyMatch(Predicate.not(Type::isStringType)))
			// this might not be a string formatting
			result = result.lub(super.fwdBinarySemantics(interprocedural, state, left, right, expressions));
		return result;
	}
}
