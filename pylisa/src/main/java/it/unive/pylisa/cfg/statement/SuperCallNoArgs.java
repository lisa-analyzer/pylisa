package it.unive.pylisa.cfg.statement;

import java.util.Set;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

/*
 * https://docs.python.org/3/library/functions.html#super
 */
public class SuperCallNoArgs extends UnresolvedCall {
	private final UnresolvedCall call;

	public SuperCallNoArgs(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, CallType.UNKNOWN, null, "super", LeftToRightEvaluation.INSTANCE, Untyped.INSTANCE, parameters);
		call = new UnresolvedCall(cfg, location, CallType.UNKNOWN, null, "super", LeftToRightEvaluation.INSTANCE, Untyped.INSTANCE);
	}

	@Override
	@SuppressWarnings("unchecked")
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Call resolved;
		try {
			// we first try with the no-parameters version since someone might have redefined super()
			// note: we only create this call if there are no parameters
			resolved = interprocedural.resolve(
					call,
					new Set[0],
					state.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
			if (resolved instanceof OpenCall) {
				Set<Type>[] ptypes = parameterTypes(expressions);
				resolved = interprocedural.resolve(
						this,
						ptypes,
						state.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
			}
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve call " + this, e);
		}

		AnalysisState<A> result = resolved.forwardSemanticsAux(interprocedural, state, params, expressions);
		getMetaVariables().addAll(resolved.getMetaVariables());
		return result;
	}
}