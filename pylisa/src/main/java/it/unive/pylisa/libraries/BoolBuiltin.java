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
import it.unive.pylisa.cfg.expression.PyNot;

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
		// the truth value for each type of x
		return PyNot.truthValue(analysis, context, state, x, false, st);
	}
}
