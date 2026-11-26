package it.unive.pylisa.libraries.svverifier;

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
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code _sv_verifier.nondet_str()}: the result is any
 * {@code str}.
 */
public class NondetStr extends PyNative {

	protected NondetStr(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "nondet_str", params);
	}

	public static NondetStr build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NondetStr(cfg, location, exprs);
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
