package it.unive.pylisa.libraries.dicts;

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
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Set;

/**
 * Native implementation of {@code dict.update(self, other, **kwargs)}, that
 * adds the entries of {@code other}, and then the keyword arguments, to the
 * dictionary, overriding the ones it has, and returns {@code None}. A
 * dictionary {@code other} is added as by {@code self |= other} (see
 * {@link DictModel#unpackInto}). Otherwise, {@code other} must be an iterable
 * of key-value pairs, whose entries are not known: any entry might be added,
 * and {@code TypeError} (for an element that is not a pair) or
 * {@code ValueError} (for a pair whose length is not 2) might be raised;
 * {@code TypeError} is surely raised if {@code other} is not iterable (see
 * {@link Iteration#iterable}). The keyword arguments are the entries of a
 * dictionary (see {@code PyAssigningStrategy}).
 */
public class DictUpdate extends PyNative {

	protected DictUpdate(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "update", params);
	}

	public static DictUpdate build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictUpdate(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression dict = args[0], other = args[1], kwargs = args[2];
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			// dictionaries are symbolic values (see DictionaryCreation)
			return compute(analysis, state, new PyNoneConstant(loc));

		AnalysisState<A> updated = state;
		if (!missing(other)) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, other, st);
			if (!types.isEmpty() && types.stream().allMatch(DictModel::isDict))
				updated = DictModel.unpackInto(analysis, state, dict, other, st, false);
			else {
				Satisfiability iterable = Iteration.iterable(analysis, state, other, st);
				updated = state.bottomExecution();
				if (iterable != Satisfiability.NOT_SATISFIED)
					// an iterable of pairs, whose entries are not known
					updated = DictModel.anyEntry(analysis, state, dict, new PushAny(Untyped.INSTANCE, loc),
							new PushAny(Untyped.INSTANCE, loc), st)
							.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR))
							.lub(raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
				else
					updated = updated.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
			}
		}
		if (!missing(kwargs))
			updated = DictModel.unpackInto(analysis, updated, dict, kwargs, st, false);
		return compute(analysis, updated, new PyNoneConstant(loc));
	}
}
