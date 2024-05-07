package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;

public class ParameterMappingRule extends MappingRule {
    int index;

    public ParameterMappingRule(int index) {
        this.index = index;
    }

    @Override
    public Expression eval(Statement statement) throws Exception {
        if (statement instanceof NaryExpression ne) {
            return ne.getSubExpressions()[index];
        }
        return null;
    }
}
