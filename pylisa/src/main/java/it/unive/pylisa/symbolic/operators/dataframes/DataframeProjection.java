package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class DataframeProjection implements TernaryOperator, DataframeOperator {

	private final int index;

	public DataframeProjection(int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return "access_rows_cols->";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> middle, Set<Type> right) {
		if (left.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Collections.emptySet();
		if (middle.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.SLICE))
				|| t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES))))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.SLICE))
				|| t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
	}
}
