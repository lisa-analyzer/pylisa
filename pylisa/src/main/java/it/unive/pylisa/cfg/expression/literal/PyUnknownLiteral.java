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
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;

/**
 * A literal whose value the analysis does not compute, such as a formatted
 * string ({@code f"..."}) or a bytes literal: it evaluates to an unknown value
 * of the given type. The source text of the literal is kept for display only.
 */
public class PyUnknownLiteral extends Literal<String> {

	/**
	 * Builds the literal.
	 *
	 * @param cfg        the CFG the literal belongs to
	 * @param location   the location of the literal
	 * @param sourceText the literal as written in the source
	 * @param type       the type of the values the literal may denote
	 */
	public PyUnknownLiteral(
			CFG cfg,
			CodeLocation location,
			String sourceText,
			Type type) {
		super(cfg, location, sourceText, type);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		return interprocedural.getAnalysis().smallStepSemantics(entryState,
				new PushAny(getStaticType(), getLocation()), this);
	}
}
