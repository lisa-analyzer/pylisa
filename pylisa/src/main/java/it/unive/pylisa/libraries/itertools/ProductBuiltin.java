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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.List;

/**
 * Native implementation of {@code itertools.product(*iterables, repeat=1)}: an
 * iterator over the tuples with an element of each iterable, {@code repeat}
 * times (see {@link ItertoolsModel}). As in python, the iterables are iterated
 * when the iterator is created: {@code TypeError} is raised if they might not
 * be iterable (see {@link Iteration#iterable}), or if {@code repeat} might not
 * be an {@code int}; {@code ValueError} might be raised if {@code repeat} might
 * be negative.
 */
public class ProductBuiltin extends ItertoolBuiltin {

	protected ProductBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "product", params);
	}

	public static ProductBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ProductBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression repeat = missing(args[1]) ? new Constant(Int32Type.INSTANCE, 1, loc) : args[1];
		Satisfiability typed = hasType(analysis, state, repeat, INT);
		List<SymbolicExpression> iterables = ListModel.exactElements(analysis, state, args[0], st);
		if (iterables == null)
			typed = typed.and(Satisfiability.UNKNOWN);
		else
			for (SymbolicExpression iterable : iterables)
				typed = typed.and(Iteration.iterable(analysis, state, iterable, st));
		AnalysisState<A> result = checked(analysis, state, typed, ItertoolsModel.PRODUCT,
				new String[] { ItertoolsModel.ITERABLES, ItertoolsModel.REPEATS },
				new SymbolicExpression[] { args[0], repeat });
		return missing(args[1]) ? result : result.lub(negative(analysis, state, repeat));
	}
}
