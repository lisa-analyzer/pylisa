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
 * The operation performed by an augmented assignment {@code x op= y} (whose
 * result is then assigned to {@code x}): it calls {@code type(x).__iop__(x, y)}
 * (e.g. {@code __iadd__}), falling back to {@code x op y} (see
 * {@link PyBinaryDispatch}) if {@code x} has no in-place method, or if it
 * returns {@code NotImplemented}.
 */
public class PyInPlaceOperation extends BinaryExpression {

	private final String iop, op, rop;

	/**
	 * Builds the operation.
	 *
	 * @param cfg    the cfg where the operation happens
	 * @param loc    the location of the operation
	 * @param symbol the symbol of the operator (e.g. {@code +=})
	 * @param iop    the in-place dunder method (e.g. {@code __iadd__})
	 * @param op     the dunder method (e.g. {@code __add__})
	 * @param rop    the reflected dunder method (e.g. {@code __radd__})
	 * @param left   the target, read
	 * @param right  the value
	 */
	public PyInPlaceOperation(
			CFG cfg,
			CodeLocation loc,
			String symbol,
			String iop,
			String op,
			String rop,
			Expression left,
			Expression right) {
		super(cfg, loc, symbol, Untyped.INSTANCE, left, right);
		this.iop = iop;
		this.op = op;
		this.rop = rop;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return iop.compareTo(((PyInPlaceOperation) o).iop);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		return PyBinaryDispatch.dispatch(interprocedural, state, expressions, this, left, right, iop, op, rop, false,
				PyBinaryDispatch.Fallback.TYPE_ERROR);
	}
}
