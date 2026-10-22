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

/**
 * Native implementation of {@code eval(source, globals, locals)}, that
 * evaluates the expression in {@code source}. The evaluated code is unknown:
 * the result is unknown, and any exception might be raised (as an
 * {@code Exception}, that might be any of its subclasses).
 */
public class Eval extends PyNative {

	protected Eval(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "eval", params);
	}

	public static Eval build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Eval(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// the evaluated expression might bind any global of the module (e.g.
		// through an assignment expression), also when it raises an exception
		AnalysisState<A> havocked = havocGlobals(analysis, state, args[1]);
		return unknown(analysis, havocked, null)
				.lub(raise(analysis, havocked, LibrarySpecificationProvider.EXCEPTION));
	}
}
