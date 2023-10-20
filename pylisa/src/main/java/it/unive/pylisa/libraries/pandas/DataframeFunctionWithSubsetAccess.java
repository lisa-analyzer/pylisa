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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.expression.NoneLiteral;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnAccess;

public class DataframeFunctionWithSubsetAccess extends it.unive.lisa.program.cfg.statement.BinaryExpression
		implements
		PluggableStatement {

	protected Statement st;

	public DataframeFunctionWithSubsetAccess(
			CFG cfg,
			CodeLocation location,
			Expression dataframe,
			Expression subset) {
		super(cfg, location, "subset-access", dataframe, subset);
	}

	public static DataframeFunctionWithSubsetAccess build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		Expression subset = null;
		if (exprs.length > 1)
			for (int i = 1; i < exprs.length; i++)
				if ((subset = getOptional(exprs[i])) != null)
					break;

		if (subset == null)
			subset = new NoneLiteral(cfg, location);
		return new DataframeFunctionWithSubsetAccess(cfg, location, exprs[0], subset);
	}

	private static Expression getOptional(
			Expression expression) {
		if (!(expression instanceof NamedParameterExpression))
			return null;

		NamedParameterExpression npe = (NamedParameterExpression) expression;
		switch (npe.getParameterName()) {
		case "subset":
			return npe.getSubExpression();
		default:
			return null;
		}
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
		if (right instanceof Constant && right.getStaticType().isNullType())
			// no subset - we access the whole dataframe
			return state.smallStepSemantics(left, st);

		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		HeapDereference derefLeft = new HeapDereference(dftype, left, location);
		BinaryExpression access = new BinaryExpression(dftype, derefLeft, right,
				ColumnAccess.INSTANCE, location);
		return state.smallStepSemantics(access, st).smallStepSemantics(left, st);
	}
}