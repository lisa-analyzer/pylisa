package it.unive.pylisa.libraries.dynamic;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code compile(source, filename, mode, ...)}, that
 * compiles the code in {@code source} into a code object, without executing it.
 * The result is unknown; {@code TypeError} is raised if the source or the file
 * name are not a {@code str} or {@code bytes}, or if the mode is not a
 * {@code str}. Errors in the compiled code (e.g. {@code SyntaxError}) are not
 * tracked.
 */
public class Compile extends PyNative {

	protected Compile(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "compile", params);
	}

	public static Compile build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Compile(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Satisfiability source = hasType(analysis, state, args[0], STR.or(BYTES));
		Satisfiability filename = hasType(analysis, state, args[1], STR.or(BYTES));
		Satisfiability mode = hasType(analysis, state, args[2], STR);
		return typeChecked(analysis, state, source.and(filename).and(mode), unknown(analysis, state, null));
	}
}
