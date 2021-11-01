package it.unive.pylisa.analysis.libraries.standardLibrary;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.pylisa.analysis.libraries.NoEffectMethod;

public class Print extends NoEffectMethod {
        public Print(CFG cfg, CodeLocation location, String constructName, Expression... parameter) {
            super(cfg, location, constructName, parameter);
        }

        public static NativeCall build(CFG cfg, CodeLocation location, Expression[] exprs) {
            return new Print(cfg, location, "print", exprs);
        }
}
