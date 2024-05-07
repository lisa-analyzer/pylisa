package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Statement;

public abstract class Condition {
    MappingRule cond;

    public Condition(MappingRule cond) {
        this.cond = cond;
    }

    public MappingRule getCond() {
        return cond;
    }

    public abstract boolean isTrue(Statement statement) throws Exception;
}
