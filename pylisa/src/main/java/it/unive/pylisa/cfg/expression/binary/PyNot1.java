package it.unive.pylisa.cfg.expression.binary;

import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.cfg.statement.NativeCall;
import it.unive.pylisa.cfg.type.PyBoolType;

public class PyNot1 extends NativeCall {

	/**
	 * Builds the logical not.
	 * 
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param sourceFile the source file name where this operation is defined
	 * @param line       the line number where this operation is defined
	 * @param col        the column where this operation is defined
	 * @param expression the operand of this operation
	 */
	public PyNot1(CFG cfg, String sourceFile, int line, int col, Expression expression) {
		super(cfg, sourceFile, line, col, "<>", PyBoolType.INSTANCE, expression);
	}
}
