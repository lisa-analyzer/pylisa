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
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.TypeHints;
import java.util.Set;

/**
 * Native implementation of
 * {@code collections.defaultdict(default_factory=None)}: a new empty
 * dictionary. With {@code default_factory} surely {@code int}, its missing keys
 * read {@code 0}, and reading one adds it (see
 * {@link it.unive.pylisa.libraries.dicts.DictModel#readDefault}); with
 * {@code None}, it is a plain dictionary, whose missing keys raise
 * {@code KeyError}. With any other factory, the dictionary is not known, and
 * {@code TypeError} might be raised (if the factory is not callable). Further
 * arguments (the initial entries) are not supported: the call does not match
 * this function.
 */
public class DefaultDictBuiltin extends PyNative {

	protected DefaultDictBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "defaultdict", params);
	}

	public static DefaultDictBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DefaultDictBuiltin(cfg, location, exprs);
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
				|| params.length != 1 || params[0].size() != 1)
			return unknown(analysis, context, state);
		SymbolicExpression factory = params[0].iterator().next();
		Set<Type> types = analysis.getRuntimeTypesOf(state, factory, context, this);
		String cls;
		if (!types.isEmpty() && types.stream().allMatch(t -> TypeHints.isBuiltin(t, "int")))
			cls = LibrarySpecificationProvider.DEFAULT_INT_DICT;
		else if (missing(factory) || hasType(analysis, context, state, factory, NONE) == Satisfiability.SATISFIED)
			cls = LibrarySpecificationProvider.DICT;
		else
			return unknown(analysis, context, state);
		// the new dictionary, allocated at a location different from the one
		// of the call (see DictBuiltin)
		DictionaryCreation creation = new DictionaryCreation(getCFG(),
				new DerivedLocation(getLocation(), "defaultdict"), cls);
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		return created;
	}

	// a dictionary that is not known, with a factory that might not be
	// callable
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknown(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state)
			throws SemanticException {
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(compute(analysis, context, state, new PushAny(Untyped.INSTANCE, getLocation())));
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
