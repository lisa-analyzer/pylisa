package it.unive.pylisa.cfg.expression;

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
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;

/**
 * Python's {@code a << b}: it calls {@code type(a).__lshift__(a, b)}, falling
 * back to {@code type(b).__rlshift__(b, a)}; if neither applies, a
 * {@code TypeError} is raised. See {@link PyBinaryDispatch} for the details.
 */
public class PyBitwiseLeftShift extends BinaryExpression {

	public PyBitwiseLeftShift(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "<<", Untyped.INSTANCE, left, right);
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
		return PyBinaryDispatch.dispatch(interprocedural, state, expressions, this, left, right,
				"__lshift__", "__rlshift__", false, PyBinaryDispatch.Fallback.TYPE_ERROR);
	}
}
