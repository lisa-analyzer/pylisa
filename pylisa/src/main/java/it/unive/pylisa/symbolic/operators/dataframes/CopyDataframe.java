package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class CopyDataframe implements UnaryOperator {

	public static final CopyDataframe INSTANCE = new CopyDataframe();

	private CopyDataframe() {
	}

	@Override
	public String toString() {
		return "copy";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> arg) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		boolean notdf = arg.stream().noneMatch(t -> t.equals(df));
		boolean notseries = arg.stream().noneMatch(t -> t.equals(series));
		if (notdf && notseries)
			return Collections.emptySet();
		if (notdf)
			return Collections.singleton(series);
		if (notseries)
			return Collections.singleton(df);
		return Set.of(df, series);
	}
}
