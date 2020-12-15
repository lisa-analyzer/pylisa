package it.unive.pylisa.cfg.expression.binary;

import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.cfg.statement.NativeCall;
import it.unive.pylisa.cfg.type.PyBoolType;

public class PyAnd extends NativeCall {

	/**
	 * Builds the logical not.
	 * 
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param sourceFile the source file name where this operation is defined
	 * @param line       the line number where this operation is defined
	 * @param col        the column where this operation is defined
	 * @param expression the operand of this operation
	 */
	public PyAnd(CFG cfg, String sourceFile, int line, int col, Expression left, Expression right) {
		super(cfg, sourceFile, line, col, "and", PyBoolType.INSTANCE, left, right);
	}
	
	
}
