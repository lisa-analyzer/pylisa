package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.NotEqual;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;

public class PyNotEqual extends NotEqual {

	public PyNotEqual(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression left, SymbolicExpression right, StatementStore<A> expressions) throws SemanticException {
		/*if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			AnalysisState<A> sem = PandasSemantics.compare(
					state,
					left,
					right,
					this,
					state.getState(),
					ComparisonOperator.NEQ);
			if (sem != null)
				return sem;
		}*/

		return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
	}

}
