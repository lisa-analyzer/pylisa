package it.unive.pylisa.libraries.svverifier;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.TypeHints;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Set;

/**
 * Native implementation of {@code _sv_verifier.check_type(value, hint)}, that
 * returns {@code None} if {@code value} has the type described by {@code hint}
 * (see {@link TypeHints}), and raises {@code TypeError} otherwise.
 */
public class CheckType extends PyNative {

	protected CheckType(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "check_type", params);
	}

	public static CheckType build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CheckType(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, args[0], context, this);
		Set<Type> hints = analysis.getRuntimeTypesOf(state, args[1], context, this);
		boolean matches = false, fails = false;
		if (types.isEmpty() || hints.isEmpty())
			matches = fails = true;
		for (Type hint : hints)
			for (Type t : types) {
				Satisfiability sat = TypeHints.satisfies(t, hint);
				matches |= sat != Satisfiability.NOT_SATISFIED;
				fails |= sat != Satisfiability.SATISFIED;
			}

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (matches)
			result = result.lub(compute(analysis, context, state, new PyNoneConstant(getLocation())));
		if (fails)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
