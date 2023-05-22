package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;

public interface Value {

	Expression toLiSAExpression(CFG init);
}
