package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;

public abstract class MappingRule {
    public abstract Expression eval(Statement statement) throws Exception;
}
