package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

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
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		// FIX ME
		/*
		 * CodeLocation location = getLocation(); PyClassType dftype =
		 * PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		 * UnaryExpression read = new UnaryExpression(dftype, arg, new
		 * ReadDataframe(0), location); return
		 * PandasSemantics.createAndInitDataframe(state, read, st);
		 */
		return state;
	}
}