package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * Native implementation of {@code input(prompt)}: the prompt can be any value
 * (it is printed with {@code str}), and the result is a {@code str} read from
 * the standard input, whose value is unknown.
 */
public class Input extends PyNative {

	protected Input(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "input", params);
	}

	public static Input build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Input(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return unknown(analysis, context, state, StringType.INSTANCE);
	}
}
