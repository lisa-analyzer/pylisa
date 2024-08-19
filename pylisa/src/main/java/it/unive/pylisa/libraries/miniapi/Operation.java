package it.unive.pylisa.libraries.miniapi;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;

public class Operation extends BinaryExpression implements PluggableStatement {

	// private Statement st;

	public Operation(
			CFG cfg,
			CodeLocation location,
			Expression statusCode,
			Expression detail) {
		super(cfg, location, "Operation", Untyped.INSTANCE, statusCode, detail);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		// this.st = st;
	}

	public static Operation build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Operation(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interproceduralAnalysis,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> statementStore)
			throws SemanticException {
		return state;
	}
}