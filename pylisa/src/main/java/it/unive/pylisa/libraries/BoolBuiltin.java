package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import java.util.Set;

/**
 * Native implementation of {@code bool(x=False)}: the truth value of {@code x},
 * as {@code not not x} (see {@link PyNot}), and {@code False} without
 * arguments. The name {@code bool} is also the type of {@code True} and
 * {@code False}, e.g. in {@code _sv_verifier.check_type(x, bool)}.
 */
public class BoolBuiltin extends PyNative {

	protected BoolBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "bool", params);
	}

	public static BoolBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BoolBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression x = args[0];
		if (missing(x))
			return compute(analysis, context, state, new Constant(BoolType.INSTANCE, false, loc));
		Set<Type> types = analysis.getRuntimeTypesOf(state, x, context, st);
		UnaryExpression notNot = new UnaryExpression(BoolType.INSTANCE,
				new UnaryExpression(BoolType.INSTANCE, x, LogicalNot.INSTANCE, loc), LogicalNegation.INSTANCE, loc);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st))
			// not not x
			return compute(analysis, context, state, notNot);

		// the truth value for each type of x, in the executions where x has
		// that type (see PyBinaryDispatch#ofType)
		AnalysisState<A> result = state.bottomExecution();
		for (Type t : types) {
			AnalysisState<A> typed = PyBinaryDispatch.ofType(analysis, context, state, x, t, types, st);
			if (typed.getExecutionState().isBottom())
				continue;
			if (t.isBooleanType())
				result = result.lub(compute(analysis, context, typed, x));
			else if (PyNot.alwaysTrue(t))
				result = result.lub(compute(analysis, context, typed, new Constant(BoolType.INSTANCE, true, loc)));
			else if (t.isPointerType())
				// e.g. an object whose class defines __bool__
				result = result.lub(compute(analysis, context, typed, new PushAny(BoolType.INSTANCE, loc)));
			else
				// not not x
				result = result.lub(compute(analysis, context, typed, notNot));
		}
		return result;
	}
}
