package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;

public class PositionalParameterMappingRule extends MappingRule {
    private int index;

    public PositionalParameterMappingRule(int index) {
        super();
        this.index = index;
    }

    public int getIndex() {
        return index;
    }

    @Override
    public Expression eval(Statement statement) throws Exception {
        if (statement instanceof NaryExpression e) {
            Expression exp = e.getSubExpressions()[index];
            if (exp instanceof NamedParameterExpression) {
                throw new Exception("Expecting a positional parameter in position " + this.index + ", found a NamedParameterExpression");
            }
            return exp;
        };
        return null;
    }
}
