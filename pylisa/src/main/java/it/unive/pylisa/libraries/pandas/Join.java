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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.ConcatCols;

public class Join extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {

	private Statement st;

	public Join(CFG cfg, CodeLocation location, Expression receiver, Expression other) {
		super(cfg, location, "join", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				receiver, other);
	}

	public static Join build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Join(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	@Override
	protected <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation loc = getLocation();
		AnalysisState<A, H, V, T> result = state.bottom();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		AnalysisState<A, H, V, T> copy = PandasSemantics.copyDataframe(state, left, st);
		ExpressionSet<SymbolicExpression> recs = copy.getComputedExpressions();
		for (SymbolicExpression rec : recs) {
			BinaryExpression cat = new BinaryExpression(dftype, rec, right,
					ConcatCols.INSTANCE, loc);
			HeapReference ref = new HeapReference(dfref, rec, loc);
			result = result.lub(copy.smallStepSemantics(cat, st).smallStepSemantics(ref, st));
		}

		return result;
	}
}
