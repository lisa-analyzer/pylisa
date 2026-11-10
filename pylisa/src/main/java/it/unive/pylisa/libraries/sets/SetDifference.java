package it.unive.pylisa.libraries.sets;

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
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.libraries.PyNative;
import java.util.List;

/**
 * Native implementation of {@code set.__sub__(self, other)} and
 * {@code set.__and__(self, other)}, that is, {@code self - other} and
 * {@code self & other} for two sets: a new set whose elements are some of the
 * ones of {@code self} (which ones is not tracked, see {@link SetModel}),
 * allocated at a location different from the one of the operation (see
 * {@link DerivedLocation}).
 */
public class SetDifference extends PyNative {

	protected SetDifference(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__sub__", params);
	}

	public static SetDifference build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetDifference(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> elements = SetModel.elements(analysis, context, state, args[0], st);
		return SetModel.create(analysis, context, state, st, new DerivedLocation(getLocation(), "set"),
				elements.getExecutionState().isBottom() ? List.of() : List.of(elements), getMetaVariables());
	}
}
