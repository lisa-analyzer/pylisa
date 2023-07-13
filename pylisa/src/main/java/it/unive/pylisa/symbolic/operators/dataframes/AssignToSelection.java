package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class AssignToSelection implements BinaryOperator, DataframeOperator {

	private final int index;

	public AssignToSelection(int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return "write";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (left.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		return Collections.singleton(series);
	}
}
