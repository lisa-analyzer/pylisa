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
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.BinaryKind;
import it.unive.pylisa.symbolic.operators.dataframes.BinaryTransform;

public class FillNA extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {

	private Statement st;

	private Axis axis = Axis.ROWS;

	private boolean inplace = false;

	public FillNA(CFG cfg, CodeLocation location, Expression series, Expression value) {
		super(cfg, location, "fillna", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				series, value);
	}

	public static FillNA build(CFG cfg, CodeLocation location, Expression[] exprs) {
		FillNA drop = new FillNA(cfg, location, exprs[0], exprs[1]);
		if (exprs.length > 2)
			for (int i = 2; i < exprs.length; i++)
				setOptional(drop, exprs[i]);
		return drop;
	}

	private static void setOptional(FillNA drop, Expression expression) {
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
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation location = getLocation();
		AnalysisState<A, H, V, T> base = state;
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		HeapDereference deref = new HeapDereference(dftype, left, location);
		ExpressionSet<SymbolicExpression> targets = new ExpressionSet<>(deref);

		if (!inplace) {
			base = PandasSemantics.copyDataframe(base, deref, st);
			targets = base.getComputedExpressions();
		}

		AnalysisState<A, H, V, T> filtered = state.bottom();
		BinaryTransform op = new BinaryTransform(0, BinaryKind.FILL_NA, axis, false);
		for (SymbolicExpression loc : targets) {
			BinaryExpression filter = new BinaryExpression(dftype, loc, right, op, location);
			SymbolicExpression ref = loc instanceof HeapDereference
					? ((HeapDereference) loc).getExpression()
					: new HeapReference(dfref, loc, location);
			filtered = filtered.lub(base.smallStepSemantics(filter, st).smallStepSemantics(ref, st));
		}

		return filtered;
	}
}
