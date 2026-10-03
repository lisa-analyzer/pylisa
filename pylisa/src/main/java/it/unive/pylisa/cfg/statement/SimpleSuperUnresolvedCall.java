package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import java.util.Set;

/*
 * https://docs.python.org/3/library/functions.html#super
 */
public class SimpleSuperUnresolvedCall extends UnresolvedCall {
	UnresolvedCall call;

	public SimpleSuperUnresolvedCall(
			CFG cfg,
			CodeLocation location,
			CallType callType,
			String qualifier,
			String targetName,
			EvaluationOrder order,
			Type staticType,
			Expression... parameters) {
		super(cfg, location, callType, qualifier, targetName, order, staticType, parameters);
		call = new UnresolvedCall(cfg, location, callType, qualifier, targetName, order, staticType);
	}

	public SimpleSuperUnresolvedCall(
			CFG cfg,
			CodeLocation location,
			CallType callType,
			String qualifier,
			String targetName,
			Expression... parameters) {
		this(cfg, location, callType, qualifier, targetName, Untyped.INSTANCE, parameters);
	}

	public SimpleSuperUnresolvedCall(
			CFG cfg,
			CodeLocation location,
			CallType callType,
			String qualifier,
			String targetName,
			Type staticType,
			Expression... parameters) {
		this(cfg, location, callType, qualifier, targetName, LeftToRightEvaluation.INSTANCE, staticType, parameters);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Call resolved;
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type>[] ptypes = parameterTypes(expressions, analysis);
		try {
			resolved = interprocedural.resolve(
					call,
					new Set[0],
					state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
			if (resolved instanceof OpenCall)
				resolved = interprocedural.resolve(
						this,
						ptypes,
						state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve call " + this, e);
		}

		AnalysisState<A> result = resolved.forwardSemanticsAux(interprocedural, state, params, expressions);
		getMetaVariables().addAll(resolved.getMetaVariables());
		return result;
	}
}