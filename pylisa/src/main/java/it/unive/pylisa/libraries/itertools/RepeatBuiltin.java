package it.unive.pylisa.libraries.itertools;

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

/**
 * Native implementation of {@code itertools.repeat(object, times=None)}: an
 * iterator over {@code object}, repeated {@code times} times, or forever (see
 * {@link ItertoolsModel}). {@code TypeError} is raised if {@code times} might
 * not be an {@code int}.
 */
public class RepeatBuiltin extends ItertoolBuiltin {

	protected RepeatBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "repeat", params);
	}

	public static RepeatBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RepeatBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Satisfiability typed = missing(args[1]) ? Satisfiability.SATISFIED : hasType(analysis, state, args[1], INT);
		return checked(analysis, state, typed, ItertoolsModel.REPEAT, new String[] { ItertoolsModel.OBJECT },
				new SymbolicExpression[] { args[0] });
	}
}
