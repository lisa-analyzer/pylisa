package it.unive.pylisa.libraries.builtins.functions;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;

public class StaticMethod extends UnaryExpression implements PluggableStatement {

	protected StaticMethod(
			CFG cfg,
			CodeLocation location,
			Expression expression) {
		super(cfg, location, "staticmethod", expression);
	}

	public static StaticMethod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StaticMethod(cfg, location, exprs[0]);
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
		// staticmethod(f) is semantically transparent for analysis:
		// return f's symbolic value so the class attribute carries
		// PyFunctionType(f),
		// enabling correct static dispatch without any implicit receiver
		// prepended.
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {

	}
}
