package it.unive.pylisa.cfg.statement;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;

/**
 * A {@link LazyEvaluatedVariadicExpression} that acts as a pass-through: when
 * applied to a function, it returns the function unchanged. Used to model
 * decorators such as {@code @cache(...)} from fastapi_cache that wrap a
 * function without changing its HTTP interface.
 */
public class PassThroughLazyExpression extends LazyEvaluatedVariadicExpression {

	public PassThroughLazyExpression(
			Type staticType,
			SymbolicExpression operand,
			UnaryOperator operator,
			CodeLocation location,
			Expression[] exprs) {
		super(staticType, operand, operator, location, exprs);
	}

	@Override
	public String toString() {
		return "PassThrough";
	}
}
