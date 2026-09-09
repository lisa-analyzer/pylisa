package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
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
import java.util.Set;

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

	@Override
	public String toString() {
		return "len";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ExpressionSet exprSet = analysis.rewrite(state, expr, st);

		for (SymbolicExpression e : exprSet) {
			Set<Type> rts = analysis.getRuntimeTypesOf(state, e, this);
			if (rts.stream().anyMatch(Type::isStringType)) {
				return state.lub(analysis.smallStepSemantics(result,
						new UnaryExpression(Int32Type.INSTANCE, e, StringLength.INSTANCE, getLocation()), this));
			}

			// String len
			return analysis.smallStepSemantics(state,
					new UnaryExpression(Int32Type.INSTANCE, expr, StringLength.INSTANCE, getLocation()), this);
		}
		// TODO handle other cases
		return state;
	}
}