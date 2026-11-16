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
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
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
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (types.isEmpty() || hints.isEmpty()) {
			// nothing is known: the value might have the type or not
			result = result.lub(compute(analysis, context, state, new PyNoneConstant(getLocation())));
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);
			return raising.moveFrom(result);
		}
		// each type of the value is checked in the executions where the value
		// has that type (see PyBinaryDispatch#ofType), so that the TypeError
		// is raised only where the value has a type not matching the hint
		for (Type t : types) {
			AnalysisState<A> typed = PyBinaryDispatch.ofType(analysis, context, state, args[0], t, types, this);
			if (typed.getExecutionState().isBottom())
				continue;
			boolean matches = false, fails = false;
			for (Type hint : hints) {
				Satisfiability sat = TypeHints.satisfies(t, hint);
				matches |= sat != Satisfiability.NOT_SATISFIED;
				fails |= sat != Satisfiability.SATISFIED;
			}
			if (matches)
				result = result.lub(compute(analysis, context, typed, new PyNoneConstant(getLocation())));
			if (fails)
				// surely raised only if no hint might match a type that can be
				// checked
				raise(raising, !matches && PyBinaryDispatch.checkable(t, types) ? Satisfiability.SATISFIED
						: Satisfiability.UNKNOWN, typed, LibrarySpecificationProvider.TYPE_ERROR);
		}
		return raising.moveFrom(result);
	}
}
