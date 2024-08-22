package it.unive.pylisa.libraries;

import java.util.Set;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.StringLength;
import it.unive.lisa.type.Type;

public class Len extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	protected Statement st;

	protected Len(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence) {
		super(cfg, location, constructName, sequence);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Len build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Len(cfg, location, "len", exprs[0]);
	}

	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		ExpressionSet exprSet = state.getState().rewrite(expr, st, state.getState());

		for (SymbolicExpression e : exprSet) {
			Set<Type> rts = state.getState().getRuntimeTypesOf(e, this, state.getState());
			if (rts.stream().anyMatch(Type::isStringType)) {
				return state.lub(result.smallStepSemantics(
						new UnaryExpression(Int32Type.INSTANCE, e, StringLength.INSTANCE, getLocation()), this));
			}

			// String len
			return state.smallStepSemantics(
					new UnaryExpression(Int32Type.INSTANCE, expr, StringLength.INSTANCE, getLocation()), this);
		}
		// TODO handle other cases
		return state;

	}

	@Override
	public String toString() {
		return "len";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}