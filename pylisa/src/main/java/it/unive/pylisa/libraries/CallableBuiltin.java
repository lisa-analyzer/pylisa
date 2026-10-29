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
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.pylisa.cfg.expression.PyValueCall;

/**
 * Native implementation of {@code callable(obj)}: {@code True} if {@code obj}
 * can be called, {@code False} if it cannot (see {@link PyValueCall#callable}),
 * and an unknown {@code bool} if both are possible, or if it is not known.
 */
public class CallableBuiltin extends PyNative {

	protected CallableBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "callable", params);
	}

	public static CallableBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CallableBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability callable = PyValueCall.callable(analysis, context, state, args[0], st);
		if (callable == Satisfiability.SATISFIED || callable == Satisfiability.NOT_SATISFIED)
			return compute(analysis, context, state,
					new Constant(BoolType.INSTANCE, callable == Satisfiability.SATISFIED, loc));
		return compute(analysis, context, state, new PushAny(BoolType.INSTANCE, loc));
	}
}
