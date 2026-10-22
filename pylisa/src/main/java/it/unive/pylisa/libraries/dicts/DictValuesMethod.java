package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code dict.values(self)}: a new view of the
 * dictionary (see {@link DictViews}).
 */
public class DictValuesMethod extends PyNative {

	protected DictValuesMethod(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "values", params);
	}

	public static DictValuesMethod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictValuesMethod(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return DictViews.create(analysis, state, args[0], LibrarySpecificationProvider.DICT_VALUES, st,
				getMetaVariables());
	}
}
