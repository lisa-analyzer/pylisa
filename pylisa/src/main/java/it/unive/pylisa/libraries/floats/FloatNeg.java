package it.unive.pylisa.libraries.floats;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;

/**
 * Native implementation of {@code float.__neg__(self)}.
 */
public class FloatNeg extends UnaryExpression implements PluggableStatement {

	protected Statement st;

	protected FloatNeg(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self) {
		super(cfg, location, constructName, self);
	}

	public static FloatNeg build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new FloatNeg(cfg, location, "__neg__", exprs[0]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
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
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new it.unive.lisa.symbolic.value.UnaryExpression(
						getStaticType(),
						expr,
						NumericNegation.INSTANCE,
						getLocation()),
				st);
	}
}
