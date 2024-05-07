package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;

public class ConditionalMapping extends MappingRule {
    Condition condition;
    MappingRule ifTrue;
    MappingRule ifFalse;

    public ConditionalMapping(Condition condition, MappingRule ifTrue, MappingRule ifFalse) {
        this.condition = condition;
        this.ifFalse = ifFalse;
        this.ifTrue = ifTrue;
    }

    @Override
    public Expression eval(Statement statement) throws Exception {
        if (condition.isTrue(statement)) {
            return ifTrue.eval(statement);
        } else {
            return ifFalse.eval(statement);
        }
    }

}
