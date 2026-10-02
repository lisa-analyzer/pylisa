package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.Equal;
import it.unive.lisa.symbolic.SymbolicExpression;

public class PyEquals extends Equal {

	public PyEquals(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		// FIX ME
		/*
		 * if (LibrarySpecificationProvider.isLibraryLoaded(
		 * LibrarySpecificationProvider.PANDAS)) { AnalysisState<A> sem =
		 * PandasSemantics.compare( state, left, right, this, state.getState(),
		 * ComparisonOperator.EQ); if (sem != null) return sem; }
		 */
		return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
	}
}
