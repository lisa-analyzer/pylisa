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
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.UnaryTransformKind;
import it.unive.pylisa.symbolic.operators.dataframes.UnaryTransform;

public class DropNA extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	private Axis axis = Axis.ROWS;

	private boolean inplace = false;

	public DropNA(
			CFG cfg,
			CodeLocation location,
			Expression series) {
		super(cfg, location, "dropna", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				series);
	}

	public static DropNA build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		DropNA drop = new DropNA(cfg, location, exprs[0]);
		if (exprs.length > 1)
			for (int i = 1; i < exprs.length; i++)
				setOptional(drop, exprs[i]);
		return drop;
	}

	private static void setOptional(
			DropNA drop,
			Expression expression) {
		if (!(expression instanceof NamedParameterExpression))
			return;

		NamedParameterExpression npe = (NamedParameterExpression) expression;
		switch (npe.getParameterName()) {
		case "axis":
			if (!(npe.getSubExpression() instanceof Int32Literal))
				return;
			drop.axis = ((Int32Literal) npe.getSubExpression()).getValue() == 0 ? Axis.ROWS : Axis.COLS;
			break;
		case "inplace":
			if (npe.getSubExpression() instanceof TrueLiteral)
				drop.inplace = true;
			else if (npe.getSubExpression() instanceof FalseLiteral)
				drop.inplace = false;
			else
				return;
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
		AnalysisState<A> base = state;
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		HeapDereference deref = new HeapDereference(dftype, expr, location);
		ExpressionSet targets = new ExpressionSet(deref);

		if (!inplace) {
			base = PandasSemantics.copyDataframe(base, deref, st);
			targets = base.getComputedExpressions();
		}

		AnalysisState<A> filtered = state.bottom();
		UnaryTransform op = new UnaryTransform(0, UnaryTransformKind.DROP_NA, axis, false);
		for (SymbolicExpression loc : targets) {
			UnaryExpression filter = new UnaryExpression(dftype, loc, op, location);
			SymbolicExpression ref = loc instanceof HeapDereference
					? ((HeapDereference) loc).getExpression()
					: new HeapReference(dfref, loc, location);
			filtered = filtered.lub(base.smallStepSemantics(filter, st).smallStepSemantics(ref, st));
		}

		return filtered;
	}
}
