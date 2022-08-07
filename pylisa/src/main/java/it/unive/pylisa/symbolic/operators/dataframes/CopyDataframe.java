package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Set;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class CopyDataframe implements UnaryOperator, DataframeOperatorWithSideEffects {

	public static final CopyDataframe INSTANCE = new CopyDataframe();

	private CopyDataframe() {
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> arg) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		boolean notdf = arg.noneMatch(t -> t.equals(df));
		boolean notseries = arg.noneMatch(t -> t.equals(series));
		if (notdf && notseries)
			return Caches.types().mkEmptySet();
		if (notdf)
			return Caches.types().mkSingletonSet(series);
		if (notseries)
			return Caches.types().mkSingletonSet(df);
		return Caches.types().mkSet(Set.of(df, series));
	}

	@Override
	public String toString() {
		return "copy";
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((UnaryExpression) container).getExpression();
	}
}
