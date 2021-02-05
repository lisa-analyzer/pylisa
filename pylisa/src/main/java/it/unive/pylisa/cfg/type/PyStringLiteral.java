package it.unive.pylisa.cfg.type;

import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Literal;
import it.unive.lisa.cfg.type.StringType;


public class PyStringLiteral  extends Literal {

	/**
	 * Builds the literal.
	 * 
	 * @param cfg        the {@link CFG} where this literal lies
	 * @param sourceFile the source file name where this literal is defined
	 * @param line       the line number where this literal is defined
	 * @param col        the column where this literal is defined
	 * @param value      the constant value represented by this literal
	 */
	public PyStringLiteral(CFG cfg, String sourceFile, int line, int col, String value) {
		super(cfg, sourceFile, line, col, value, PyStringType.INSTANCE);
	}
}
