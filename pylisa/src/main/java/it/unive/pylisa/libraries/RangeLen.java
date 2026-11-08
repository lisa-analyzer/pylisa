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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;

/**
 * Native implementation of {@code Range.__len__(self)} (see
 * {@link RangeBuiltin}): the length of a range is not tracked, so it is an
 * unknown int.
 */
public class RangeLen extends PyNative {

	protected RangeLen(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__len__", params);
	}

	public static RangeLen build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RangeLen(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// the length of a range of a known number of elements (the largest
		// length looked for is the one of the lists placed exactly, see
		// ListMul)
		Integer length = RangeBuiltin.length(analysis, context, state, args[0], this, 32);
		if (length != null)
			return compute(analysis, context, state, new Constant(Int32Type.INSTANCE, length, getLocation()));
		return UnknownLength.of(analysis, context, state, this, getMetaVariables());
	}
}
