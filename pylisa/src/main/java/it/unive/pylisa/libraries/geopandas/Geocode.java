package it.unive.pylisa.libraries.geopandas;

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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.Enumerations.UnaryReshapeKind;
import it.unive.pylisa.symbolic.operators.dataframes.UnaryReshape;

public class Geocode extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	public Geocode(
			CFG cfg,
			CodeLocation location,
			Expression dataframe) {
		super(cfg, location, "geocode", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				dataframe);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Geocode build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Geocode(cfg, location, exprs[0]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		UnaryReshape op = new UnaryReshape(0, UnaryReshapeKind.TO_GEOCODE);
		return PandasSemantics.applyUnary(state, expr, st, op);
	}
}
