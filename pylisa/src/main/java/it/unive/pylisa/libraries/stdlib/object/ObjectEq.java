package it.unive.pylisa.libraries.stdlib.object;

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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;

public class ObjectEq extends BinaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectEq(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, "__eq__", cfg.getDescriptor().getUnit().getProgram().getTypes().getBooleanType(), left,
				right);
	}

	public static ObjectEq build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectEq(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		return state.smallStepSemantics(
				new it.unive.lisa.symbolic.value.BinaryExpression(
						getStaticType(),
						left,
						right,
						ComparisonEq.INSTANCE,
						getLocation()),
				st);
	}
}
