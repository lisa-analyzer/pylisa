package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;

public class Mapping {
    String target;
    CFG cfg;

    MappingRule rule;

    public Mapping(String target, MappingRule rule) {
        this.target = target;
        this.rule = rule;
    }


    public Expression eval(Statement statement) {
        return new StringLiteral(statement.getCFG(), SyntheticLocation.INSTANCE, "GET");
    }

}
