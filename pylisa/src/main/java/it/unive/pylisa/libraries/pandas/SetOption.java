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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.type.VoidType;

public class SetOption extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {
	private Statement st;

	protected SetOption(CFG cfg, CodeLocation location, String constructName, Expression right, Expression left) {
		super(cfg, location, constructName, VoidType.INSTANCE, left, right);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	public static SetOption build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new SetOption(cfg, location, "set_option", exprs[0], exprs[1]);
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
		CodeLocation location = getLocation();

		BinaryExpression bin = new BinaryExpression(VoidType.INSTANCE, left, right,
				it.unive.pylisa.symbolic.operators.SetOption.INSTANCE, location);
		AnalysisState<A, H, V, T> binState = state.smallStepSemantics(bin, st);
		// the skip is to cleanup the expression on the stack
		return binState.smallStepSemantics(new Skip(location), st);
	}
}