package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;

public class ROSLisaAnalysis<A extends AbstractState<A>> {
	public SymbolicExpression getSymbolicExpression() {
		return symbolicExpression;
	}

	public Statement getStatement() {
		return statement;
	}

	public void setStatement(
			Statement statement) {
		this.statement = statement;
	}

	public AnalysisState<A> getAnalysisState() {
		return analysisState;
	}

	private SymbolicExpression symbolicExpression;
	private Statement statement;

	AnalysisState<A> analysisState;

	public ROSLisaAnalysis(
			SymbolicExpression symbolicExpression,
			Statement statement,
			AnalysisState<A> analysisState) {
		this.symbolicExpression = symbolicExpression;
		this.statement = statement;
		this.analysisState = analysisState;
	}
}
