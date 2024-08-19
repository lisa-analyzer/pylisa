package it.unive.pylisa.libraries.miniapi;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;

public class Init extends UnaryExpression implements PluggableStatement {

	// private Statement st;

	public Init(
			CFG cfg,
			CodeLocation location,
			Expression operation) {
		super(cfg, location, "Init", operation);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		// this.st = st;
	}

	public static Init build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		if (exprs.length != 1) {
			throw new IllegalArgumentException(
					"ExecuteOperation requires exactly one argument: the operation to execute");
		}
		return new Init(cfg, location, exprs[0]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression operation,
			StatementStore<A> expressions)
			throws SemanticException {
		return state;
	}
}