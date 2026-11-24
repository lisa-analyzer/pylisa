package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.pylisa.symbolic.PyFloatConstant;

/**
 * A Python float literal: its value is the 32-bits number used by the analyses,
 * and its semantics is a {@link PyFloatConstant} also keeping Python's (64-bits)
 * value of the literal.
 */
public class PyFloatLiteral extends Float32Literal {

	private final double exact;

	/**
	 * Builds the literal.
	 * 
	 * @param cfg      the {@link CFG} where this literal lies
	 * @param location the location where this literal is defined
	 * @param exact    Python's value of the literal
	 */
	public PyFloatLiteral(
			CFG cfg,
			CodeLocation location,
			double exact) {
		super(cfg, location, (float) exact);
		this.exact = exact;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		return interprocedural.getAnalysis().smallStepSemantics(entryState,
				new PyFloatConstant(exact, getLocation()), interprocedural.getCurrentContext(), this);
	}
}
