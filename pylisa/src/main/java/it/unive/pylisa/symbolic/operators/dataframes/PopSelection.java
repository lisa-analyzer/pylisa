package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PopSelection implements UnaryOperator {

	public static final PopSelection INSTANCE = new PopSelection();

	private PopSelection() {
	}

	@Override
	public String toString() {
		return "pop_selection";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (argument.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		return Collections.singleton(df);
	}
}
