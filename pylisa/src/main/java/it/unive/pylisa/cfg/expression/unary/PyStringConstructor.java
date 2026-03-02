package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.value.StringConstructor;
import java.util.Set;

public class PyStringConstructor extends it.unive.lisa.program.cfg.statement.UnaryExpression {

	public PyStringConstructor(
			CFG cfg,
			SourceCodeLocation location,
			Expression exp) {
		super(cfg, location, "str", exp);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {

		Set<Type> rt = interprocedural.getAnalysis().getRuntimeTypesOf(state, expr, this);
		if (rt.stream().anyMatch(Type::isStringType) || rt.stream().anyMatch(Type::isNumericType)) {
			return interprocedural.getAnalysis().smallStepSemantics(state,
					new UnaryExpression(StringType.INSTANCE, expr, StringConstructor.INSTANCE, getLocation()), this);
		}
		return null;
	}
}
