package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
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
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.FilterNull;
import it.unive.pylisa.symbolic.operators.FilterNull.Axis;

public class DropNA extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	private Axis axis = Axis.ROWS;

	private boolean inplace = false;

	public DropNA(CFG cfg, CodeLocation location, String constructName, Expression series) {
		super(cfg, location, constructName, series);
	}

	public static DropNA build(CFG cfg, CodeLocation location, Expression[] exprs) {
		DropNA drop = new DropNA(cfg, location, "dropna", exprs[0]);
		if (exprs.length > 1)
			for (int i = 1; i < exprs.length; i++)
				setOptional(drop, exprs[i]);
		return drop;
	}

	private static void setOptional(DropNA drop, Expression expression) {
		if (!(expression instanceof NamedParameterExpression))
			return;

		NamedParameterExpression npe = (NamedParameterExpression) expression;
		switch (npe.getParameterName()) {
		case "axis":
			if (!(npe.getSubExpression() instanceof Int32Literal))
				return;
			drop.axis = ((Int32Literal) npe.getSubExpression()).getValue() == 0 ? Axis.ROWS : Axis.COLUMNS;
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
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression expr,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation location = getLocation();
		AnalysisState<A, H, V, T> base = state;
		HeapDereference deref = new HeapDereference(PandasDataframeType.INSTANCE, expr, location);
		ExpressionSet<SymbolicExpression> targets = new ExpressionSet<>(deref);

		if (!inplace) {
			base = PandasSemantics.copyDataframe(base, deref, st);
			targets = base.getComputedExpressions();
		}

		AnalysisState<A, H, V, T> filtered = state.bottom();
		FilterNull op = new FilterNull(axis);
		for (SymbolicExpression loc : targets) {
			UnaryExpression filter = new UnaryExpression(PandasDataframeType.INSTANCE, loc, op, location);
			HeapReference ref = new HeapReference(PandasDataframeType.REFERENCE, loc, location);
			filtered = filtered.lub(base.smallStepSemantics(filter, st).smallStepSemantics(ref, st));
		}

		return filtered;
	}
}
