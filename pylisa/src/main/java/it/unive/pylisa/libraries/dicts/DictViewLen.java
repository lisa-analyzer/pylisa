package it.unive.pylisa.libraries.dicts;

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
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code __len__(self)} of the views of a dictionary
 * (see {@link DictViews}): the length of the dictionary, if it is known (see
 * {@link DictModel#length}), an unknown int otherwise.
 */
public class DictViewLen extends PyNative {

	protected DictViewLen(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__len__", params);
	}

	public static DictViewLen build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictViewLen(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		for (Type t : analysis.getRuntimeTypesOf(state, args[0], context, this)) {
			if (!DictViews.isView(t))
				continue;
			AnalysisState<A> dicts = DictViews.dictionaries(analysis, context, state, args[0], t, st);
			for (SymbolicExpression dict : dicts.getExecutionExpressions()) {
				Integer length = DictModel.length(analysis, context, dicts, dict, st);
				result = result.lub(length == null ? unknown(analysis, context, dicts, Int32Type.INSTANCE)
						: compute(analysis, context, dicts, new Constant(Int32Type.INSTANCE, length, getLocation())));
			}
		}
		return result.getExecutionState().isBottom() ? unknown(analysis, context, state, Int32Type.INSTANCE) : result;
	}
}
