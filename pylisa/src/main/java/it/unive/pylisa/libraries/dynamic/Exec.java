package it.unive.pylisa.libraries.dynamic;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code exec(source, globals, locals)}, that executes
 * the code in {@code source}. The executed code is unknown: the result is
 * {@code None}, and any exception might be raised (as an {@code Exception},
 * that might be any of its subclasses). The variables that the code might
 * assign are not tracked.
 */
public class Exec extends PyNative {

	protected Exec(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "exec", params);
	}

	public static Exec build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Exec(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state, new PyNoneConstant(getLocation()))
				.lub(raise(analysis, state, LibrarySpecificationProvider.EXCEPTION));
	}
}
