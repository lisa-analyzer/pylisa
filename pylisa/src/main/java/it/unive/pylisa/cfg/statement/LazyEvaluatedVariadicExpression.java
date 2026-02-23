package it.unive.pylisa.cfg.statement;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.nary.NaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;

public class LazyEvaluatedVariadicExpression extends it.unive.lisa.symbolic.value.UnaryExpression {
    VariadicExpression expr;
    Expression[] additionalExpressions;
    /**
     * Builds the binary expression.
     *
     * @param staticType the static type of this expression
     * @param operator   the operator to apply
     * @param location   the code location of the statement that has generated
     *                   this expression
     */
    public LazyEvaluatedVariadicExpression(Type staticType, SymbolicExpression operand, UnaryOperator operator, CodeLocation location, Expression[] exprs) {
        super(staticType, operand, operator, location);
        this.additionalExpressions = exprs;
    }


    @Override
    public String toString() {
        return "LazyEval(" + expr + ")";
    }



}
