package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.statement.Statement;

public class IsInstanceOf extends Condition {

    Class clazz;
    public IsInstanceOf(MappingRule cond, Class clazz) {
        super(cond);
        this.clazz = clazz;
    }

    @Override
    public boolean isTrue(Statement statement) throws Exception {
        Statement s = this.getCond().eval(statement);
        return clazz.isInstance(s);
    }
}
