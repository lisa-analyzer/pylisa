package it.unive.pylisa.libraries.collections;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.dicts.DictModel;

/**
 * Native implementation of {@code collections.Counter(iterable, **kwargs)}: a
 * new, empty dictionary of class {@code Counter} (allocated at a location
 * different from the one of the call, see {@link DerivedLocation}), whose
 * missing keys have the value {@code 0} (see {@link DictModel#MISSING}). With
 * initial elements or entries, the counter is unknown, and any exception might
 * be raised (see {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}).
 */
public class CounterBuiltin extends DefaultDictBuiltin {

	protected CounterBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "Counter", LibrarySpecificationProvider.COUNTER, params);
	}

	public static CounterBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CounterBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return create(analysis, context, state, new Constant(Int32Type.INSTANCE, 0, getLocation()), true, args[0],
				args[1]);
	}
}
