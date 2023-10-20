package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.LessOrEqual;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.dataframes.ComparisonOperator;

public class PyLessOrEqual extends LessOrEqual {

	public PyLessOrEqual(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> sem = PandasSemantics.compare(
				state,
				left,
				right,
				this,
				state.getState(),
				ComparisonOperator.LEQ);
		if (sem != null)
			return sem;

		// python does not require the types to be numeric
		return state.smallStepSemantics(
				new BinaryExpression(
						BoolType.INSTANCE,
						left,
						right,
						PyComparisonLe.INSTANCE,
						getLocation()),
				this);
	}
}
