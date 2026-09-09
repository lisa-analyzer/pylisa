package it.unive.pylisa.libraries.fastapi.sarl;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;

public class AddRoute extends TernaryExpression implements PluggableStatement {

	// private Statement st;

	public AddRoute(
			CFG cfg,
			CodeLocation location,
			Expression app,
			Expression path,
			Expression handler) {
		super(cfg, location, "add_route", Untyped.INSTANCE, app, path, handler);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		// this.st = st;
	}

	public static AddRoute build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new AddRoute(cfg, location, exprs[0], exprs[1], exprs[2]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdTernarySemantics(
			InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression left,
			SymbolicExpression middle, SymbolicExpression right, StatementStore<A> expressions)
			throws SemanticException {
		return state;
	}
}