package it.unive.pylisa.analysis.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.caches.Caches;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.DataFrameConstant;

public class ReadCsv extends BinaryExpression implements PluggableStatement {
	private Statement st;

	protected ReadCsv(CFG cfg, CodeLocation location, String constructName, Type staticType, Expression left,
			Expression right) {
		super(cfg, location, constructName, staticType, left, right);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	public static ReadCsv build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new ReadCsv(cfg, location, "read_csv", PyDataframeType.INSTANCE, exprs[0], exprs[1]);
	}

	@Override
	protected <A extends AbstractState<A, H, V>, H extends HeapDomain<H>, V extends ValueDomain<V>> AnalysisState<A, H,
			V> binarySemantics(InterproceduralAnalysis<A, H, V> interprocedural, AnalysisState<A, H, V> state,
					SymbolicExpression left, SymbolicExpression right, StatementStore<A, H, V> expressions)
					throws SemanticException {
		AnalysisState<A, H, V> afterAlloc = state.smallStepSemantics(
				new HeapAllocation(Caches.types().mkSingletonSet(PyDataframeType.INSTANCE), this.getLocation()), st);
		DataFrameConstant constant = new DataFrameConstant(PyDataframeType.INSTANCE, right, this.getLocation());
		AnalysisState<A, H, V> assigned = null;
		for (SymbolicExpression exp : afterAlloc.getComputedExpressions()) {
			if (assigned == null)
				assigned = afterAlloc.assign(exp, constant, st);
			else
				assigned = assigned.lub(afterAlloc.assign(exp, constant, st));
		}
		return new AnalysisState<A, H, V>(assigned.getState(), afterAlloc.getComputedExpressions());
	}
}