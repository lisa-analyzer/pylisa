package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;

/**
 * Python's {@code abs(x)}: it calls {@code type(x).__abs__(x)} (a
 * {@code bool} uses the one of {@code int}), raising {@code TypeError} if the
 * class of {@code x} does not define it (e.g. {@code abs("a")}). See
 * {@link PyBinaryDispatch#dispatchUnary}.
 */
public class PyAbs extends UnaryExpression {

	/**
	 * Builds the call.
	 *
	 * @param cfg      the cfg where the call is
	 * @param location the location of the call
	 * @param arg      the argument
	 */
	public PyAbs(
			CFG cfg,
			CodeLocation location,
			Expression arg) {
		super(cfg, location, "abs", Untyped.INSTANCE, arg);
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
		return PyBinaryDispatch.dispatchUnary(interprocedural, state, expressions, this, expr, "__abs__");
	}
}
