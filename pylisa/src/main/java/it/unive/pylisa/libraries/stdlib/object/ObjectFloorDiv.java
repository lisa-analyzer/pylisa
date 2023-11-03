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
import it.unive.pylisa.cfg.expression.literal.PyNotImplementedLiteral;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ObjectFloorDiv extends BinaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectFloorDiv(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg,
				location,
				"__floordiv__",
				PyClassType.lookup(LibrarySpecificationProvider.OBJECT),
				left,
				right);
	}

	public static ObjectFloorDiv build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectFloorDiv(cfg, location, exprs[0], exprs[1]);
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
		return new PyNotImplementedLiteral(getCFG(), getLocation()).semantics(state, interprocedural, expressions);
	}
}
