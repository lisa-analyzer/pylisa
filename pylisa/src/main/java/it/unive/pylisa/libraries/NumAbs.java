package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;

/**
 * Native implementation of {@code int.__abs__(self)} and
 * {@code float.__abs__(self)}: {@code self} if it is not negative, and
 * {@code -self} otherwise.
 */
public class NumAbs extends UnaryExpression implements PluggableStatement {

	protected Statement st;

	protected NumAbs(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self) {
		super(cfg, location, constructName, self);
	}

	public static NumAbs build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NumAbs(cfg, location, "__abs__", exprs[0]);
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		Constant zero = new Constant(Int32Type.INSTANCE, 0, loc);
		AnalysisState<A> positive = analysis.assume(state,
				new BinaryExpression(BoolType.INSTANCE, expr, zero, ComparisonGe.INSTANCE, loc), context, st, st);
		AnalysisState<A> negative = analysis.assume(state,
				new BinaryExpression(BoolType.INSTANCE, expr, zero, ComparisonLt.INSTANCE, loc), context, st, st);
		AnalysisState<A> result = state.bottomExecution();
		if (!positive.getExecutionState().isBottom())
			result = result.lub(analysis.smallStepSemantics(positive, expr, context, st));
		if (!negative.getExecutionState().isBottom())
			result = result.lub(analysis.smallStepSemantics(negative,
					new it.unive.lisa.symbolic.value.UnaryExpression(getStaticType(), expr,
							NumericNegation.INSTANCE, loc),
					context, st));
		return result;
	}
}
