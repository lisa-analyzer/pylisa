package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;

public class NamedParameterMappingRule extends MappingRule {
    String name;

    public NamedParameterMappingRule(String name) {
        this.name = name;
    }
    @Override
    public Expression eval(Statement statement) throws Exception {
        if (statement instanceof NaryExpression ne) {
            return getNamedParameterExpression(ne);
        }
        return null;
    }

    public Expression getNamedParameterExpression(NaryExpression expr) throws Exception {
        for (Expression e: expr.getSubExpressions()) {
            if (e instanceof NamedParameterExpression ne && ne.getParameterName().equals(name)) {
                return ne;
            }
        }
        throw new Exception("A NamedParameterExpression with name " + name + "does not exists in statement.");
    }
}
