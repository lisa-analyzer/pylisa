package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.List;
import java.util.Set;

/**
 * Native implementation of {@code tuple(iterable=())}: {@code tuple()} is a new
 * empty tuple, and {@code tuple(t)} is {@code t} itself when it is a tuple. For
 * a list or a tuple whose elements are known (see
 * {@link ListModel#exactElements}), the new tuple has the same elements, in
 * order; for any other iterable, it is a tuple whose length and elements are
 * not known. The new tuples are allocated at a location different from the one
 * of the call (see {@link DerivedLocation}). {@code TypeError} is raised if
 * {@code iterable} might not be iterable (see {@link Iteration#iterable}). The
 * name {@code tuple} is also the type of tuples, e.g. in
 * {@code _sv_verifier.check_type(x, tuple)}.
 */
public class TupleBuiltin extends PyNative {

	protected TupleBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "tuple", params);
	}

	public static TupleBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new TupleBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = new DerivedLocation(getLocation(), "tuple");
		SymbolicExpression iterable = args[0];
		if (missing(iterable))
			return TupleCreation.create(analysis, state, st, loc, new ExpressionSet[0], getMetaVariables());

		Set<Type> types = analysis.getRuntimeTypesOf(state, iterable, st);
		if (!types.isEmpty() && types.stream().allMatch(Unpacking::isTuple))
			// a tuple is not copied
			return compute(analysis, state, iterable);

		Satisfiability it = Iteration.iterable(analysis, state, iterable, st);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		if (it != Satisfiability.NOT_SATISFIED) {
			List<SymbolicExpression> elements = ListModel.exactElements(analysis, state, iterable, st);
			if (elements != null) {
				ExpressionSet[] values = new ExpressionSet[elements.size()];
				for (int i = 0; i < values.length; i++)
					values[i] = new ExpressionSet(elements.get(i));
				result = TupleCreation.create(analysis, state, st, loc, values, getMetaVariables());
			} else {
				// a tuple whose length (and elements) are not known
				AnalysisState<A> created = TupleCreation.create(analysis, state, st, loc, new ExpressionSet[0],
						getMetaVariables());
				for (SymbolicExpression tuple : created.getExecutionExpressions())
					result = result.lub(TupleCreation.unknownLength(analysis, created, tuple, st, loc));
			}
		}
		if (it != Satisfiability.SATISFIED)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
