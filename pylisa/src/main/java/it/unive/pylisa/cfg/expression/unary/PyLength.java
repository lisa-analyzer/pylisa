package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code len(x)} builtin. It invokes {@code type(x).__len__(x)}.
 * {@code __len__} is registered as a static-style method for native types
 * (e.g. {@code str}, mirroring how {@code int}/{@code float} dunders are
 * registered) and as an instance method for {@code Sequence} subtypes
 * ({@code list}, {@code set}, {@code dict}, {@code tuple}, {@code slice}), so
 * both call kinds are attempted per runtime type. There is no reflected
 * method, so if {@code __len__} does not resolve either way for a given
 * runtime type, that type simply does not contribute to the result
 * (mirroring the {@code TypeError} Python would raise, which is not
 * explicitly modeled here).
 */
public class PyLength extends it.unive.lisa.program.cfg.statement.UnaryExpression {

	public PyLength(
			CFG cfg,
			SourceCodeLocation location,
			Expression exp) {
		super(cfg, location, "len", exp);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rts = analysis.getRuntimeTypesOf(state, expr, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type t : rts) {
			UnresolvedCall len = null;
			for (CallType kind : new CallType[] { CallType.STATIC, CallType.INSTANCE }) {
				UnresolvedCall candidate = new UnresolvedCall(
						getCFG(),
						getLocation(),
						kind,
						null,
						"__len__",
						LeftToRightEvaluation.INSTANCE,
						getSubExpression());
				try {
					interprocedural.resolve(candidate, new Set[] { Collections.singleton(t) }, aliasing);
					len = candidate;
					break;
				} catch (CallResolutionException e) {
					// try the next call kind
				}
			}

			if (len == null)
				// this type does not support len(): it does not contribute to the result
				continue;
			result = result.lub(len.forwardSemantics(state, interprocedural, expressions));
		}

		return result;
	}

}