package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class AccessKeys implements UnaryOperator {

	public static final AccessKeys INSTANCE = new AccessKeys();

	private AccessKeys() {
	}

	@Override
	public String toString() {
		return "keys()";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> arg) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (arg.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		return Collections.singleton(df);
	}
}
