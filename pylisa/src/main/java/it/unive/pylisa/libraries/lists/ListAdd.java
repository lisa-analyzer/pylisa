package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
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
import it.unive.pylisa.UnsupportedStatementException;

/**
 * Native implementation of {@code list.__add__(self, other)}: list
 * concatenation ({@code [1] + [2]}). Not implemented: allocating a fresh
 * list and copying elements into it would require a domain that can
 * summarize "all elements of this allocation" independent of a constant
 * index, which this codebase does not have; a naive fresh allocation was
 * tried and produced allocation-site aliasing with sibling list literals
 * instead of a genuinely independent object, which is worse than failing
 * loudly.
 */
public class ListAdd extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected ListAdd(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static ListAdd build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListAdd(cfg, location, "__add__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		throw new UnsupportedStatementException(this);
	}
}
