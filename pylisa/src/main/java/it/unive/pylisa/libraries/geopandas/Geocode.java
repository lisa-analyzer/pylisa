package it.unive.pylisa.libraries.geopandas;

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
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.PopSelection;

public class Geocode extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

	private Statement st;

	public Geocode(CFG cfg, CodeLocation location, Expression dataframe) {
		super(cfg, location, "geocode", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				dataframe);
	}

	public static Geocode build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Geocode(cfg, location, exprs[0]);
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
		AnalysisState<A, H, V, T> result = state.bottom();
		SymbolicExpression dataframe = ((AccessChild) expr).getContainer();

		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		Type dfref = ((PyClassType) dftype).getReference();
		PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);

		HeapAllocation allocation = new HeapAllocation(dftype, location);
		AnalysisState<A, H, V, T> allocated = state.smallStepSemantics(allocation, st);
		AnalysisState<A, H, V, T> copy = state.bottom();
		for (SymbolicExpression loc : allocated.getComputedExpressions()) {
			// copy the dataframe
			AnalysisState<A, H, V, T> assigned = allocated.assign(loc, dataframe, st);
			for (SymbolicExpression id : assigned.getComputedExpressions()) {
				UnaryExpression transform = new UnaryExpression(seriestype, id,
						new ApplyTransformation(ApplyTransformation.Kind.TO_GEOCODE),
						location);
				copy = copy.lub(assigned.smallStepSemantics(transform, st));
			}

			// we leave a reference to the fresh dataframe on the stack
			HeapReference ref = new HeapReference(dfref, loc, location);
			UnaryExpression pop = new UnaryExpression(dftype, dataframe, PopSelection.INSTANCE,
					location);
			result = result.lub(copy.smallStepSemantics(pop, st).smallStepSemantics(ref, st));
		}

		return result;
	}
}
