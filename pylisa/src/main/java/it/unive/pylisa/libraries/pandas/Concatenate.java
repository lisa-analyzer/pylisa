package it.unive.pylisa.libraries.pandas;

import java.util.List;

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
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.AtomList;
import it.unive.pylisa.symbolic.operators.ConcatRows;

public class Concatenate extends UnaryExpression implements PluggableStatement {

	private Statement st;

	public Concatenate(CFG cfg, CodeLocation location, Expression list) {
		super(cfg, location, "concat", PandasDataframeType.REFERENCE, list);
	}

	public static Concatenate build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Concatenate(cfg, location, exprs[0]);
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
		CodeLocation loc = getLocation();
		PushAny pushany = new PushAny(PandasDataframeType.REFERENCE, loc);
		if (!(expr instanceof AtomList))
			return state.smallStepSemantics(pushany, st);

		AnalysisState<A, H, V, T> result = state.bottom();
		AtomList atoms = (AtomList) expr;
		List<ExpressionSet<ValueExpression>> list = atoms.getList();
		if (list.isEmpty())
			// unknown
			return state.smallStepSemantics(pushany, st);
		else if (list.size() == 1) {
			for (ValueExpression e : list.iterator().next())
				result = result.lub(state.smallStepSemantics(e, st));
			return result;
		}

		for (ValueExpression start : list.iterator().next()) {
			AnalysisState<A, H, V, T> base = PandasSemantics.copyDataframe(state, start, st);
			ExpressionSet<SymbolicExpression> recs = base.getComputedExpressions();
			for (int i = 1; i < list.size(); i++) {
				AnalysisState<A, H, V, T> concat = state.bottom();
				for (SymbolicExpression rec : recs)
					for (ValueExpression arg : list.get(i)) {
						BinaryExpression cat = new BinaryExpression(PandasDataframeType.INSTANCE, rec, arg,
								ConcatRows.INSTANCE, loc);
						HeapReference ref = new HeapReference(PandasDataframeType.INSTANCE, rec, loc);
						concat = concat.lub(base.smallStepSemantics(cat, st).smallStepSemantics(ref, st));
					}
				base = concat;
			}
			result = result.lub(base);
		}

		return result;
	}
}
