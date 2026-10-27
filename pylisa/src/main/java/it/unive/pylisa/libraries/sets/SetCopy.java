package it.unive.pylisa.libraries.sets;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.libraries.PyNative;
import java.util.List;

/**
 * Native implementation of {@code set.copy(self)}, that returns a new set with
 * the same elements (see {@link SetModel}), allocated at a location different
 * from the one of the call (see {@link DerivedLocation}).
 */
public class SetCopy extends PyNative {

	protected SetCopy(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "copy", params);
	}

	public static SetCopy build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetCopy(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> elements = SetModel.elements(analysis, state, args[0], st);
		CodeLocation loc = new DerivedLocation(getLocation(), "copy");
		return SetModel.create(analysis, state, st, loc,
				elements.getExecutionState().isBottom() ? List.of() : List.of(elements), getMetaVariables());
	}
}
