package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code dict.get(self, key, default=None)}: the value
 * of the key, or {@code default} if the dictionary does not have it (see
 * {@link DictModel#get}). {@code TypeError} is raised if the key might not be
 * hashable.
 */
public class DictGet extends PyNative {

	protected DictGet(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "get", params);
	}

	public static DictGet build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictGet(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			// dictionaries are symbolic values (see DictionaryCreation)
			return compute(analysis, context, state, new PushAny(Untyped.INSTANCE, getLocation()));
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		return raising.moveFrom(DictModel.get(analysis, context, state, args[0], args[1], orNone(args[2]), st,
				raising));
	}
}
