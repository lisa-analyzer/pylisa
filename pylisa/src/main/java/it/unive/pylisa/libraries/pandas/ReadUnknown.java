package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.analysis.dataframes.symbolic.ReadDataframe;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ReadUnknown
		extends
		// this could be a simple expression, but the casting in nativecfg
		// forces this superclass
		it.unive.lisa.program.cfg.statement.NaryExpression
		implements
		PluggableStatement {

	private Statement st;

	public ReadUnknown(
			CFG cfg,
			CodeLocation location) {
		super(cfg,
				location,
				"read unknown",
				PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				new Expression[0]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	public static ReadUnknown build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReadUnknown(cfg, location);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		return 0;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		UnaryExpression read = new UnaryExpression(dftype,
				new PushAny(getCFG().getDescriptor().getUnit().getProgram().getTypes().getStringType(), location),
				new ReadDataframe(0), location);
		return PandasSemantics.createAndInitDataframe(state, read, st);
	}
}