package it.unive.pylisa.cfg.expression.comparison;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.comparison.Equal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;

/**
 * Python's {@code a == b}: it calls {@code type(a).__eq__(a, b)}, falling back
 * to {@code type(b).__eq__(b, a)}; if neither applies, the result is
 * {@code False} (comparing by identity). See {@link PyBinaryDispatch} for the
 * details. Comparisons involving pandas objects are handled by
 * {@code PandasSemantics} beforehand.
 */
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			AnalysisState<A> sem = PandasSemantics.compare(
					analysis,
					state,
					left,
					right,
					this,
					ComparisonOperator.EQ);
			if (sem != null)
				return sem;
		}

		return PyBinaryDispatch.dispatch(interprocedural, state, expressions, this, left, right,
				"__eq__", "__eq__", true, PyBinaryDispatch.Fallback.FALSE);
	}
}
