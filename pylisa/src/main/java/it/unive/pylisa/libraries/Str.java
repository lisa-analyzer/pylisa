package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import java.util.Set;

public class Str extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	protected Statement st;

	protected Str(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence) {
		super(cfg, location, constructName, sequence);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression expr, StatementStore<A> expressions) throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rts = analysis.getRuntimeTypesOf(state, expr, this);
		if (rts.stream().anyMatch(Type::isStringType) || rts.stream().anyMatch(Type::isNumericType)) {
			return analysis.smallStepSemantics(
					state,
					new UnaryExpression(StringType.INSTANCE, expr, StringConstructor.INSTANCE, getLocation()), this);
		}
		// TODO Handle other cases
		return state;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Str build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Str(cfg, location, "str", exprs[0]);
	}


	@Override
	public String toString() {
		return "str";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}