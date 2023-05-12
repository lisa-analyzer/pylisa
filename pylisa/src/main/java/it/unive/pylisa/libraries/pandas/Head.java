package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRows;
import it.unive.pylisa.symbolic.operators.dataframes.ProjectRows;

public class Head extends BinaryExpression implements PluggableStatement {

	private Statement st;

	public Head(CFG cfg, CodeLocation location, Expression dataframe, Expression n) {
		super(cfg, location, "head", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				dataframe, n);
	}

	public static Head build(CFG cfg, CodeLocation location, Expression[] exprs) {
		if (exprs.length == 2)
			return new Head(cfg, location, exprs[0], exprs[1]);
		else
			// 5 is the default from pandas
			return new Head(cfg, location, exprs[0], new Int32Literal(cfg, location, 5));
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
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();

		HeapDereference derefLeft = new HeapDereference(dftype, left, location);
		Constant start = new Constant(Int32Type.INSTANCE, 0, location);

		// we allocate the copy that will have only the given rows
		AnalysisState<A, H, V, T> copied = PandasSemantics.copyDataframe(state, derefLeft, st);

		// the receiver will have its rows accessed instead
		TernaryExpression access = new TernaryExpression(dftype, derefLeft, start, right,
				AccessRows.INSTANCE, location);
		AnalysisState<A, H, V, T> accessed = copied.smallStepSemantics(access, st);

		AnalysisState<A, H, V, T> result = state.bottom();
		for (SymbolicExpression id : copied.getComputedExpressions()) {
			// the new dataframe will have its rows projected
			TernaryExpression projection = new TernaryExpression(dftype, id, start, right,
					ProjectRows.INSTANCE, location);
			AnalysisState<A, H, V, T> tmp = accessed.smallStepSemantics(projection, st);

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(dfref, id, location);
			result = result.lub(tmp.smallStepSemantics(ref, st));
		}

		return result;
	}
}
