package it.unive.pylisa.libraries.fastapi.helpers;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;

public class SemanticsHelpers {
    public static NamedParameterExpression getNamedParameterExpr(
            Expression[] subExpressions,
            String name) {
        for (Expression e : subExpressions) {
            if (e instanceof NamedParameterExpression
                    && ((NamedParameterExpression) e).getParameterName().equals(name)) {
                return ((NamedParameterExpression) e);
            }
        }
        return null;
    }
}
