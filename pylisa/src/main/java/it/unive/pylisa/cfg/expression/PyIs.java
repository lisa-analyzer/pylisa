package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * The Python comparison {@code left is right}: whether the two operands are the
 * same object. Identity with {@code None} is decided from the values of the
 * operands, since {@code None} is a singleton; the identity of other objects is
 * not tracked, and the comparison is then an unknown boolean.
 */
public class PyIs extends BinaryExpression {

	public PyIs(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "is", BoolType.INSTANCE, left, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		SymbolicExpression identity;
		if (left instanceof PyNoneConstant || right instanceof PyNoneConstant)
			// None is a singleton: an object is None exactly when it equals
			// None, which the value domains decide
			identity = new it.unive.lisa.symbolic.value.BinaryExpression(BoolType.INSTANCE, left, right,
					ComparisonEq.INSTANCE, getLocation());
		else
			// the identity of other objects is not tracked
			identity = new PushAny(BoolType.INSTANCE, getLocation());
		return interprocedural.getAnalysis().smallStepSemantics(state, identity, this);
	}
}
