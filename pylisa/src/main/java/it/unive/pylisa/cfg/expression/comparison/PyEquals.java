package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.Equal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;

public class PyEquals extends Equal {

	public PyEquals(
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
		try {
			AnalysisState<A> sem = PandasSemantics.compare(
					state,
					left,
					right,
					this,
					state.getState(),
					ComparisonOperator.EQ);
			if (sem != null)
				return sem;
		} catch (Exception e) {
			return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
		}
		return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
	}
}
