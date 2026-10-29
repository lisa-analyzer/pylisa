package it.unive.pylisa.libraries;

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
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.compare.IsInstance;
import java.util.Set;

/**
 * Native implementation of Python's {@code isinstance(obj, classinfo)}: whether
 * {@code obj} has the type described by {@code classinfo} (see
 * {@link TypeHints}). It raises {@code TypeError} if {@code classinfo} is not a
 * type. When the result is not known, it is the {@link IsInstance} test, whose
 * branches refine the types of {@code obj}.
 */
public class IsInstanceBuiltin extends PyNative {

	protected IsInstanceBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "isinstance", params);
	}

	public static IsInstanceBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IsInstanceBuiltin(cfg, location, exprs);
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
		boolean maybeTrue = false, maybeFalse = false, raises = false;
		if (hints.isEmpty())
			maybeTrue = maybeFalse = raises = true;
		for (Type hint : hints) {
			Satisfiability type = TypeHints.isType(hint);
			raises |= type != Satisfiability.SATISFIED;
			if (type == Satisfiability.NOT_SATISFIED)
				continue;
			if (types.isEmpty())
				maybeTrue = maybeFalse = true;
			for (Type t : types) {
				Satisfiability sat = TypeHints.satisfies(t, hint);
				maybeTrue |= sat != Satisfiability.NOT_SATISFIED;
				maybeFalse |= sat != Satisfiability.SATISFIED;
			}
		}

		CodeLocation loc = getLocation();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (maybeTrue && maybeFalse)
			// the branches of a condition refine the types of obj
			result = compute(analysis, context, state,
					new BinaryExpression(BoolType.INSTANCE, args[0], args[1], IsInstance.INSTANCE, loc));
		else if (maybeTrue || maybeFalse)
			result = compute(analysis, context, state, new Constant(BoolType.INSTANCE, maybeTrue, loc));
		if (raises)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
