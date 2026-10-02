package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyExceptionType;
import java.util.Set;

/**
 * The arithmetic of Python numbers between two operands. Python's numbers
 * include booleans, which count as the integers {@code 0} and {@code 1}: the
 * operation is computed where each operand may be a number or a boolean.
 * Operands of other types (lists, tuples, objects defining the special
 * methods) are not modelled: they give an unknown result, and may raise
 * {@code TypeError}. A division or a remainder by a divisor that may be zero
 * may raise {@code ZeroDivisionError}.
 */
final class NumericOperands {

	private NumericOperands() {
	}

	/**
	 * Computes the semantics of an arithmetic operation that cannot divide by
	 * zero.
	 *
	 * @param <A>             the kind of abstract state
	 * @param <D>             the kind of abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state before the operation
	 * @param left            the first operand
	 * @param right           the second operand
	 * @param operator        the operator
	 * @param statement       the statement performing the operation
	 *
	 * @return the state after the operation
	 *
	 * @throws SemanticException if the operation cannot be computed
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> apply(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			BinaryOperator operator,
			Expression statement)
			throws SemanticException {
		return apply(interprocedural, state, left, right, operator, false, statement);
	}

	/**
	 * Computes the semantics of an arithmetic operation.
	 *
	 * @param <A>             the kind of abstract state
	 * @param <D>             the kind of abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state before the operation
	 * @param left            the first operand
	 * @param right           the second operand
	 * @param operator        the operator, or {@code null} when the result of
	 *                            the operation on numbers is not computed
	 * @param divides         whether the operation divides by its second
	 *                            operand
	 * @param statement       the statement performing the operation
	 *
	 * @return the state after the operation
	 *
	 * @throws SemanticException if the operation cannot be computed
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> apply(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			BinaryOperator operator,
			boolean divides,
			Expression statement)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> leftTypes = analysis.getRuntimeTypesOf(state, left, statement);
		Set<Type> rightTypes = analysis.getRuntimeTypesOf(state, right, statement);
		AnalysisState<A> result = state.bottomExecution();
		if (leftTypes.stream().anyMatch(NumericOperands::isNumber)
				&& rightTypes.stream().anyMatch(NumericOperands::isNumber)) {
			Satisfiability byZero = Satisfiability.NOT_SATISFIED;
			if (divides) {
				byZero = analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, right,
						new Constant(Int32Type.INSTANCE, 0, statement.getLocation()), ComparisonEq.INSTANCE,
						statement.getLocation()), statement);
				if (byZero != Satisfiability.NOT_SATISFIED)
					result = result.lub(raise(analysis, state, PyExceptionType.ZERO_DIVISION_ERROR, statement));
			}
			if (byZero != Satisfiability.SATISFIED) {
				SymbolicExpression computed = operator == null
						? new PushAny(statement.getStaticType(), statement.getLocation())
						: new BinaryExpression(statement.getStaticType(), left, right, operator,
								statement.getLocation());
				result = result.lub(analysis.smallStepSemantics(state, computed, statement));
			}
		}
		if (leftTypes.isEmpty() || rightTypes.isEmpty()
				|| !leftTypes.stream().allMatch(NumericOperands::isNumber)
				|| !rightTypes.stream().allMatch(NumericOperands::isNumber)) {
			// other operands (lists, tuples, objects defining the special
			// methods) are not modelled: their result is unknown, and the
			// operation may be undefined for them
			result = result.lub(analysis.smallStepSemantics(state,
					new PushAny(Untyped.INSTANCE, statement.getLocation()), statement));
			result = result.lub(raise(analysis, state, PyExceptionType.TYPE_ERROR, statement));
		}
		return result;
	}

	/**
	 * Yields the state where the statement raises an exception.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param analysis  the analysis
	 * @param state     the state before the statement
	 * @param type      the type of the exception
	 * @param statement the statement
	 *
	 * @return the state
	 *
	 * @throws SemanticException if the error cannot be recorded
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> raise(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			PyExceptionType type,
			Expression statement)
			throws SemanticException {
		return analysis.moveExecutionToError(state, new AnalysisState.Error(type, statement), statement);
	}

	/**
	 * Yields whether values of a type are Python numbers.
	 *
	 * @param type the type
	 *
	 * @return {@code true} for numeric types and booleans
	 */
	static boolean isNumber(
			Type type) {
		return type.isNumericType() || type.isBooleanType();
	}
}
