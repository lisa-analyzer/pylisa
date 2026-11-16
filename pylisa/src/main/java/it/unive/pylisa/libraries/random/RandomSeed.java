package it.unive.pylisa.libraries.random;

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
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Set;

/**
 * Native implementation of {@code random.seed(a=None)}, that returns
 * {@code None}: since python 3.11, the seed must be {@code None}, an
 * {@code int}, a {@code float}, a {@code str} or a {@code bytes}, and any other
 * value (e.g. a tuple, or an object) raises {@code TypeError}.
 */
public class RandomSeed extends PyNative {

	protected RandomSeed(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "seed", params);
	}

	public static RandomSeed build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RandomSeed(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, args[0], context, this);
		boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (unknown || types.stream().anyMatch(RandomSeed::seed))
			result = compute(analysis, context, state, new PyNoneConstant(getLocation()));
		if (unknown || !types.stream().allMatch(RandomSeed::seed))
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}

	// whether the values of the type are valid seeds
	private static boolean seed(
			Type t) {
		return t.isNullType() || t.isNumericType() || t.isBooleanType() || t.isStringType()
				|| t instanceof PyBytesType;
	}
}
