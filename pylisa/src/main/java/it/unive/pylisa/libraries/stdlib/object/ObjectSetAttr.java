package it.unive.pylisa.libraries.stdlib.object;

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
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ObjectSetAttr extends TernaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectSetAttr(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression middle,
			Expression right) {
		super(cfg,
				location,
				"__setattr__",
				PyClassType.lookup(LibrarySpecificationProvider.OBJECT),
				left,
				middle,
				right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static ObjectSetAttr build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectSetAttr(cfg, location, exprs[0], exprs[1], exprs[2]);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdTernarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression middle,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		// TODO given that we handle assignments separately, do we need to do
		// something here?
		return state;
	}
}
