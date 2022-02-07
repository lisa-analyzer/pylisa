package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.type.common.Int32;
import it.unive.pylisa.symbolic.ProjectRows;

public class Head extends BinaryExpression implements PluggableStatement {

	private Statement st;

	public Head(CFG cfg, CodeLocation location, String constructName, Expression dataframe, Expression n) {
		super(cfg, location, constructName, dataframe, n);
	}

	public static Head build(CFG cfg, CodeLocation location, Expression[] exprs) {
		if (exprs.length == 2)
			return new Head(cfg, location, "head", exprs[0], exprs[1]);
		else
			// 5 is the default from pandas
			return new Head(cfg, location, "head", exprs[0], new Int32Literal(cfg, location, 5));
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		Constant start = new Constant(Int32.INSTANCE, 0, getLocation());
		TernaryExpression head = new TernaryExpression(PyDataframeType.INSTANCE, left, start, right,
				ProjectRows.INSTANCE, getLocation());
		AnalysisState<A, H, V, T> headState = state.smallStepSemantics(head, st);
		HeapReference ref = new HeapReference(PyDataframeType.REFERENCE, left, getLocation());
		return headState.smallStepSemantics(ref, st);
	}
}
