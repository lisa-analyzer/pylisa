package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.numeric.Multiplication;
import it.unive.lisa.symbolic.SymbolicExpression;

/**
 * Python's {@code a * b}: it calls {@code type(a).__mul__(a, b)}, falling back
 * to {@code type(b).__rmul__(b, a)}; if neither applies, a {@code TypeError} is
 * raised. See {@link PyBinaryDispatch} for the details.
 */
public class PyMultiplication extends Multiplication {

	public PyMultiplication(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
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
				"__mul__", "__rmul__", false, PyBinaryDispatch.Fallback.TYPE_ERROR);
	}
}
