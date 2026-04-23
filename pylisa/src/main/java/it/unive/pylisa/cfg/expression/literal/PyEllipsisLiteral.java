package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.symbolic.PyEllipsisConstant;

public class PyEllipsisLiteral extends Literal<Object> {

	public PyEllipsisLiteral(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location, null, Untyped.INSTANCE);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		return interprocedural.getAnalysis().smallStepSemantics(
				entryState, new PyEllipsisConstant(getLocation()), this);
	}
}
