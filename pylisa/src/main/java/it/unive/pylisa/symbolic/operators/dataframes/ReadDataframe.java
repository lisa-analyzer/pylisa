package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class ReadDataframe implements UnaryOperator {

	public static final ReadDataframe INSTANCE = new ReadDataframe();

	private ReadDataframe() {
	}

	@Override
	public String toString() {
		return "read_df";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		if (argument.stream().noneMatch(Type::isStringType))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
	}
}
