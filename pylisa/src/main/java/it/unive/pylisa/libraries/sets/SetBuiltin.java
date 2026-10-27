package it.unive.pylisa.libraries.sets;

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
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import java.util.List;

/**
 * Native implementation of {@code set(iterable)}: a new set (see
 * {@link SetModel}), allocated at a location different from the one of the call
 * (see {@link DerivedLocation}), whose elements are the ones of
 * {@code iterable} (see {@link Iteration#elements}), or none without
 * {@code iterable}. {@code TypeError} is raised if {@code iterable} might not
 * be iterable (see {@link Iteration#iterable}), and might be raised if its
 * elements might not be hashable (which is not checked). When pandas is used,
 * the set is not known. The name {@code set} is also the type of sets, e.g. in
 * {@code _sv_verifier.check_type(x, set)}.
 */
public class SetBuiltin extends PyNative {

	protected SetBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "set", params);
	}

	public static SetBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = new DerivedLocation(getLocation(), "set");
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			return compute(analysis, state, new PushAny(Untyped.INSTANCE, getLocation()));
		SymbolicExpression iterable = args[0];
		if (missing(iterable))
			return SetModel.create(analysis, state, st, loc, List.of(), getMetaVariables());
		Satisfiability it = Iteration.iterable(analysis, state, iterable, st);
		AnalysisState<A> result = state.bottomExecution();
		if (it != Satisfiability.NOT_SATISFIED) {
			Expression[] subs = getSubExpressions();
			AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 0 ? subs[0] : null,
					iterable, st, getMetaVariables());
			result = SetModel.create(analysis, elements, st, loc, List.of(elements), getMetaVariables());
		}
		if (it != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
