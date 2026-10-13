package it.unive.pylisa.libraries;

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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.unary.PyLength;

public class Len extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	protected Statement st;

	protected Len(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence) {
		super(cfg, location, constructName, sequence);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Len build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Len(cfg, location, "len", exprs[0]);
	}

	@Override
	public String toString() {
		return "len";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		// the same semantics of len(x) when called directly (see PyLength)
		PyLength length = new PyLength(getCFG(), getLocation(), getSubExpression());
		AnalysisState<A> result = length.length(interprocedural, state, expr, expressions, st == null ? this : st);
		getMetaVariables().addAll(length.getMetaVariables());
		return result;
	}
}
