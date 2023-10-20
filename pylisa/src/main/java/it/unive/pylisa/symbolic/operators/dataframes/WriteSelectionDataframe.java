package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class WriteSelectionDataframe implements BinaryOperator {

	public static final WriteSelectionDataframe INSTANCE = new WriteSelectionDataframe();

	private WriteSelectionDataframe() {
	}

	@Override
	public String toString() {
		return "write";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (left.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		return Collections.singleton(series);
	}
}
