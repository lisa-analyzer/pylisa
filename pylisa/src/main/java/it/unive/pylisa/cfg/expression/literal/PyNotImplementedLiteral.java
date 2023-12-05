package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.cfg.type.PyNotImplementedType;
import it.unive.pylisa.symbolic.PyNotImplementedConstant;

public class PyNotImplementedLiteral extends Literal<Object> {

	public PyNotImplementedLiteral(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location, null, PyNotImplementedType.INSTANCE);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		return entryState.smallStepSemantics(new PyNotImplementedConstant(getLocation()), this);
	}
}
