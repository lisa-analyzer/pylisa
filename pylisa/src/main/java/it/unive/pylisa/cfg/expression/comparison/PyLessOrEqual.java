package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.LessOrEqual;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;

public class PyLessOrEqual extends LessOrEqual {

	public PyLessOrEqual(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression left, SymbolicExpression right, StatementStore<A> expressions) throws SemanticException {
		// python does not require the types to be numeric
		// FIX ME
		/*if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			AnalysisState<A> sem = PandasSemantics.compare(
					state,
					left,
					right,
					this,
					state.getState(),
					ComparisonOperator.LEQ);
			if (sem != null)
				return sem;
		}*/

		return interprocedural.getAnalysis().smallStepSemantics(
				state,
				new BinaryExpression(
						BoolType.INSTANCE,
						left,
						right,
						PyComparisonLe.INSTANCE,
						getLocation()),
				this);
	}
}
