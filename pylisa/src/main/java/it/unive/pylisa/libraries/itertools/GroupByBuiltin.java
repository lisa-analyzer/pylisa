package it.unive.pylisa.libraries.itertools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.Iteration;

/**
 * Native implementation of {@code itertools.groupby(iterable, key=None)}: an
 * iterator over the tuples of a key and the group of the elements of
 * {@code iterable} with that key (see {@link ItertoolsModel}), where the key of
 * an element is {@code key(element)}, or the element itself without
 * {@code key}. {@code TypeError} is raised if {@code iterable} might not be
 * iterable (see {@link Iteration#iterable}).
 */
public class GroupByBuiltin extends ItertoolBuiltin {

	protected GroupByBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "groupby", params);
	}

	public static GroupByBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new GroupByBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return checked(analysis, state, Iteration.iterable(analysis, state, args[0], st), ItertoolsModel.GROUPBY,
				new String[] { ItertoolsModel.ITERABLE, ItertoolsModel.KEY },
				new SymbolicExpression[] { args[0], args[1] });
	}
}
