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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.CreateDataframe;

public class DataFrame extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	private Statement st;

	public DataFrame(CFG cfg, CodeLocation location, Expression arg) {
		super(cfg, location, "DataFrame", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				arg);
	}

	@Override
	final public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

	public static DataFrame build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new DataFrame(cfg, location, exprs[0]);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> unarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression arg,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		UnaryExpression read = new UnaryExpression(dftype, arg, CreateDataframe.INSTANCE, location);
		return PandasSemantics.createAndInitDataframe(state, read, st);
	}
}