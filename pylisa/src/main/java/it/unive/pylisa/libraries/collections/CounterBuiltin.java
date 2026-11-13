package it.unive.pylisa.libraries.collections;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code collections.Counter(iterable)}: with no
 * argument, a new empty dictionary whose missing keys read {@code 0}, without
 * being added (see
 * {@link it.unive.pylisa.libraries.dicts.DictModel#readDefault}). Counting the
 * elements of {@code iterable} is not supported: the result is not known, and
 * {@code TypeError} might be raised (if {@code iterable} is not iterable).
 * Keyword arguments are not supported: the call does not match this function.
 */
public class CounterBuiltin extends PyNative {

	protected CounterBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "Counter", params);
	}

	public static CounterBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CounterBuiltin(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		ScopeId<A> context = interprocedural.getCurrentContext();
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)
				|| params.length != 1 || params[0].size() != 1 || !missing(params[0].iterator().next())) {
			// the elements of the iterable are not counted
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
			return raising.moveFrom(compute(analysis, context, state, new PushAny(Untyped.INSTANCE, getLocation())));
		}
		// the new dictionary, allocated at a location different from the one
		// of the call (see DictBuiltin)
		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(getLocation(), "counter"),
				LibrarySpecificationProvider.COUNTER);
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		return created;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// not used: see forwardSemanticsAux
		return compute(analysis, context, state, new PushAny(Untyped.INSTANCE, getLocation()));
	}
}
