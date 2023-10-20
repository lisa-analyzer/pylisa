package it.unive.pylisa.libraries.pandas;

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
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;

public class ReadCsv extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	private Statement st;

	public ReadCsv(
			CFG cfg,
			CodeLocation location,
			Expression arg) {
		super(cfg, location, "read_csv", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				arg);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	public static ReadCsv build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReadCsv(cfg, location, exprs[0]);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression arg,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		UnaryExpression read = new UnaryExpression(dftype, arg, ReadDataframe.INSTANCE, location);
		return PandasSemantics.createAndInitDataframe(state, read, st);
	}
}