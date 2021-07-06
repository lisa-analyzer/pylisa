package it.unive.pylisa.cfg.expression.binary;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.callgraph.CallGraph;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.program.cfg.statement.BinaryNativeCall;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.type.PyBoolType;

/**
 * A Python or function call (e1 | e2, e1 or e2).
 * 
 * @author Nicolò Barbato
 */

public class PyOr extends BinaryNativeCall{

	/**
	 * Builds a Python or expression at a given location in the program.
	 * 
	 * @param cfg           the cfg that this expression belongs to
	 * @param sourceFile    the source file where this expression happens. If
	 *                      unknown, use {@code null}
	 * @param line          the line number where this expression happens in the
	 *                      source file. If unknown, use {@code -1}
	 * @param col           the column where this expression happens in the source
	 *                      file. If unknown, use {@code -1}
	 * @param exp1		    left-hand side operand
	 * @param exp2		    right-hand side operand
	 */
	
	public PyOr(CFG cfg, CodeLocation loc, Expression left, Expression right, String token) {
		super(cfg, loc, token.equals("or")? "or" : "|", PyBoolType.INSTANCE, left, right);
	}

	@Override
	protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H, V> binarySemantics(AnalysisState<A, H, V> entryState, CallGraph callGraph, AnalysisState<A, H, V> leftState, SymbolicExpression leftExp, AnalysisState<A, H, V> rightState, SymbolicExpression rightExp) throws SemanticException {
		throw new UnsupportedStatementException();
	}
}
