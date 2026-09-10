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
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;

/**
 * Python's bitwise inversion ({@code ~x}). It invokes
 * {@code type(x).__invert__(x)}. There is no reflected method (there is only
 * one operand), so if {@code __invert__} does not resolve for a given runtime
 * type, that type simply does not contribute to the result.
 */
public class PyBitwiseNot extends UnaryExpression {

	public PyBitwiseNot(
			CFG cfg,
			CodeLocation loc,
			Expression expression) {
		super(cfg, loc, "~", Untyped.INSTANCE, expression);
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
			UnresolvedCall invert = new UnresolvedCall(
					getCFG(),
					getLocation(),
					CallType.STATIC,
					null,
					"__invert__",
					LeftToRightEvaluation.INSTANCE,
					getSubExpression());
			try {
				interprocedural.resolve(invert, new Set[] { Collections.singleton(t) }, aliasing);
			} catch (CallResolutionException e) {
				// unsupported operand type: this type does not contribute
				continue;
			}
			result = result.lub(invert.forwardSemantics(state, interprocedural, expressions));
		}

		return result;
	}
}
