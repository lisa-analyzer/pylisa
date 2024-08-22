package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
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
import it.unive.pylisa.analysis.dataframes.symbolic.UnaryTransform;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.Axis;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.UnaryTransformKind;

public class Apply extends BinaryExpression implements PluggableStatement {

	private Statement st;

	public Apply(
			CFG cfg,
			CodeLocation location,
			Expression dataframe,
			Expression lambda) {
		super(cfg, location, "apply", dataframe.getStaticType(), dataframe, lambda);
	}

	public static Apply build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Apply(cfg, location, exprs[0], exprs[1]);
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

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		UnaryTransform op = new UnaryTransform(0, UnaryTransformKind.LAMBDA, Axis.ROWS, right);
		return PandasSemantics.applyUnary(state, left, st, op);
	}
}
