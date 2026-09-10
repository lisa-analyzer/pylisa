package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.numeric.Negation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * Python's unary minus ({@code -x}). It is a separate operation from
 * subtraction: it invokes {@code type(x).__neg__(x)}. There is no reflected
 * method (there is only one operand), so if {@code __neg__} does not resolve
 * for a given runtime type, that type simply does not contribute to the result.
 */
public class PyNegation extends Negation {

	/**
	 * Builds the negation.
	 *
	 * @param cfg      the {@link CFG} where this operation lies
	 * @param location the location where this literal is defined
	 * @param expr     the operand of this operation
	 */
	public PyNegation(
			CFG cfg,
			CodeLocation location,
			Expression expr) {
		super(cfg, location, expr);
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
			UnresolvedCall neg = new UnresolvedCall(
					getCFG(),
					getLocation(),
					CallType.STATIC,
					null,
					"__neg__",
					LeftToRightEvaluation.INSTANCE,
					getSubExpression());
			try {
				interprocedural.resolve(neg, new Set[] { Collections.singleton(t) }, aliasing);
			} catch (CallResolutionException e) {
				// TODO: unsupported operand type: this type does not
				// contribute, we should throw an exception here
				continue;
			}
			result = result.lub(neg.forwardSemantics(state, interprocedural, expressions));
		}

		return result;
	}
}
