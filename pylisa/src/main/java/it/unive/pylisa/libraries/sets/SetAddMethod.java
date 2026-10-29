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
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code set.add(self, elem)}, that adds {@code elem}
 * to the elements of the set (a weak update, see {@link SetModel#add}) and
 * returns {@code None}. Whether {@code elem} is hashable is not checked.
 */
public class SetAddMethod extends PyNative {

	protected SetAddMethod(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "add", params);
	}

	public static SetAddMethod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SetAddMethod(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> added = SetModel.add(analysis, context, state, args[0], args[1], st);
		return compute(analysis, context, added, new PyNoneConstant(getLocation()));
	}
}
