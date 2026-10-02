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
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.symbolic.operators.PythonArithmetic;

/**
 * Python's unary {@code -x} and {@code +x}. On numbers they are the product
 * with {@code -1} and {@code 1}, which is exact for integers and floats
 * (including the sign of zero) and turns booleans into integers, as Python
 * does; on other values they may raise {@code TypeError}.
 */
public class PyUnaryArithmetic extends UnaryExpression {

	private final int sign;

	/**
	 * Builds the expression.
	 *
	 * @param cfg      the CFG the expression belongs to
	 * @param location the location of the expression
	 * @param operand  the operand
	 * @param negated  {@code true} for {@code -x}, {@code false} for
	 *                     {@code +x}
	 */
	public PyUnaryArithmetic(
			CFG cfg,
			CodeLocation location,
			Expression operand,
			boolean negated) {
		super(cfg, location, negated ? "-" : "+", operand);
		this.sign = negated ? -1 : 1;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return Integer.compare(sign, ((PyUnaryArithmetic) o).sign);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression operand,
			StatementStore<A> expressions)
			throws SemanticException {
		return NumericOperands.apply(interprocedural, state, operand,
				new Constant(Int32Type.INSTANCE, sign, getLocation()), PythonArithmetic.Mul.INSTANCE, this);
	}
}
