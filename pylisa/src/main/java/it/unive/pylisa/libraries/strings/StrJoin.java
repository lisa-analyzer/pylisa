package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.lists.ListModel;

/**
 * Native implementation of {@code str.join(iterable)}: a {@code str}, whose
 * value is not tracked. {@code TypeError} is raised if the iterable might not
 * be iterable (see {@link Iteration#iterable}), or if one of its elements might
 * not be a {@code str} (see {@link Iteration#elements}). An empty iterable
 * yields the empty {@code str}, also when the elements it might have are not
 * {@code str}s: the result is computed unless the iterable is a list that
 * surely has an element, and all its elements surely are not {@code str}s.
 */
public class StrJoin extends PyNative {

	protected StrJoin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "join", params);
	}

	public static StrJoin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrJoin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression iterable = args[1];
		Satisfiability iterates = Iteration.iterable(analysis, state, iterable, st);

		// the types of the elements: the state of the iteration is only used to
		// read them, as the iteration itself does not happen here
		boolean mightBeStr = false, mightNotBeStr = false;
		if (iterates != Satisfiability.NOT_SATISFIED) {
			Expression[] subs = getSubExpressions();
			AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 1 ? subs[1] : null,
					iterable, st, getMetaVariables());
			for (SymbolicExpression element : elements.getExecutionExpressions()) {
				Satisfiability str = hasType(analysis, elements, element, STR);
				mightBeStr |= str != Satisfiability.NOT_SATISFIED;
				mightNotBeStr |= str != Satisfiability.SATISFIED;
			}
		}
		// whether the iterable surely has an element
		boolean nonEmpty = analysis.getRuntimeTypesOf(state, iterable, st).stream().allMatch(ListModel::isList)
				&& ListModel.shape(analysis, state, iterable, st).prefix() > 0;

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		if (iterates != Satisfiability.NOT_SATISFIED && (mightBeStr || !nonEmpty))
			result = result.lub(compute(analysis, state, new PushAny(StringType.INSTANCE, getLocation())));
		if (iterates != Satisfiability.SATISFIED || mightNotBeStr)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
