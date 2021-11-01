package it.unive.pylisa.analysis.libraries.pandas;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.pylisa.analysis.libraries.NoEffectMethod;

public class Tail extends NoEffectMethod {
        public Tail(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
            super(cfg, location, constructName, parameter);
        }

        public static NativeCall build(CFG cfg, CodeLocation location, Expression[] exprs) {
            return new Tail(cfg, location, "tail", exprs);
        }
}
