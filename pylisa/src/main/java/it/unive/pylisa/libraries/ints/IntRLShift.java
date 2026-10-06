package it.unive.pylisa.libraries.ints;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseShiftLeft;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

/**
 * Native implementation of {@code int.__rlshift__(self, other)}, i.e. the
 * reflected left shift {@code other << self}. Unlike addition/multiplication,
 * shifting is not commutative, so this computes {@code right << left} rather
 * than {@code left << right} (the caller binds {@code self} to {@code left} and
 * {@code other} to {@code right}, following the same argument order used for
 * {@link IntLShift}).
 */
public class IntRLShift extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected IntRLShift(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static IntRLShift build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IntRLShift(cfg, location, "__rlshift__", exprs[0], exprs[1]);
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		it.unive.lisa.symbolic.value.BinaryExpression shift = new it.unive.lisa.symbolic.value.BinaryExpression(
				Int32Type.INSTANCE, right, left, BitwiseShiftLeft.INSTANCE, loc);
		// python raises ValueError for a negative shift count
		it.unive.lisa.symbolic.value.BinaryExpression negativeCount = new it.unive.lisa.symbolic.value.BinaryExpression(
				BoolType.INSTANCE, left, new Constant(Int32Type.INSTANCE, 0, loc), ComparisonLt.INSTANCE, loc);
		return ExceptionGuard.guardedCompute(interprocedural.getAnalysis(), state, negativeCount,
				LibrarySpecificationProvider.VALUE_ERROR, shift, getCFG(), loc, st, this);
	}
}
