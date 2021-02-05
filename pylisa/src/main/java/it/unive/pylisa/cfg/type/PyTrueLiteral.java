package it.unive.pylisa.cfg.type;

import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Literal;
import it.unive.lisa.symbolic.types.BoolType;

public class PyTrueLiteral extends Literal {

	/**
	 * Builds the literal.
	 * 
	 * @param cfg        the {@link CFG} where this literal lies
	 * @param sourceFile the source file name where this literal is defined
	 * @param line       the line number where this literal is defined
	 * @param col        the column where this literal is defined
	 */
	public PyTrueLiteral(CFG cfg, String sourceFile, int line, int col) {
		super(cfg, sourceFile, line, col, true, BoolType.INSTANCE);
	}
}
