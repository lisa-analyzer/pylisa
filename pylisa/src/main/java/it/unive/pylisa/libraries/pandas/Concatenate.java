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
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.dataframes.AxisConcatenation;

public class Concatenate extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	private Axis axis = Axis.ROWS;

	public Concatenate(
			CFG cfg,
			CodeLocation location,
			Expression list) {
		super(cfg, location, "concat", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(), list);
	}

	public static Concatenate build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		Concatenate concat = new Concatenate(cfg, location, exprs[0]);
		if (exprs.length > 1)
			for (int i = 1; i < exprs.length; i++)
				setOptional(concat, exprs[i]);
		return concat;
	}

	private static void setOptional(
			Concatenate concat,
			Expression expression) {
		if (!(expression instanceof NamedParameterExpression))
			return;

		NamedParameterExpression npe = (NamedParameterExpression) expression;
		switch (npe.getParameterName()) {
		case "axis":
			if (!(npe.getSubExpression() instanceof Int32Literal))
				return;
			concat.axis = ((Int32Literal) npe.getSubExpression()).getValue() == 0 ? Axis.ROWS : Axis.COLS;
			break;
		default:
			return;
		}
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
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		UnaryExpression concat = new UnaryExpression(dftype, expr, new AxisConcatenation(0, axis), location);
		return PandasSemantics.createAndInitDataframe(state, concat, st);
	}
}
