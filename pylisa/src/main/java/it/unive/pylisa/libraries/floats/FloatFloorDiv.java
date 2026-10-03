package it.unive.pylisa.libraries.floats;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
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
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.libraries.DivisionGuard;
import it.unive.pylisa.symbolic.operators.FloorDivision;

/**
 * Native implementation of {@code float.__floordiv__(self, other)}.
 * <p>
 * {@code other} (the divisor) is checked against {@code 0.0} via
 * {@link DivisionGuard}: a {@code ZeroDivisionError} is raised when it is
 * (possibly) zero, mirroring real Python.
 */
public class FloatFloorDiv extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected FloatFloorDiv(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static FloatFloorDiv build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new FloatFloorDiv(cfg, location, "__floordiv__", exprs[0], exprs[1]);
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();

		it.unive.lisa.symbolic.value.BinaryExpression div = new it.unive.lisa.symbolic.value.BinaryExpression(
				getStaticType(), left, right, FloorDivision.INSTANCE, loc);
		Constant zero = new Constant(Float32Type.INSTANCE, 0f, loc);
		return DivisionGuard.guardedCompute(analysis, state, right, zero, div, getCFG(), loc, st, this);
	}
}
