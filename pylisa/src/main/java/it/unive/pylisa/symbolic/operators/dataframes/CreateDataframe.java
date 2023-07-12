package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class CreateDataframe implements UnaryOperator, DataframeOperator {

	private final int index;

	public CreateDataframe(int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return "create_df";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
		if (argument.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.DICT))))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
	}
}
