package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;

public class StringConstantMappingRule extends ConstantMappingRule {
    String value;

    public StringConstantMappingRule(String value) {
        this.value = value;
    }

    @Override
    public Expression eval(Statement statement) {
        return new StringLiteral(statement.getCFG(), SyntheticLocation.INSTANCE, value);
    }
}
