package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class ColumnProjection implements BinaryOperator, DataframeOperator {

	private final int index;

	public ColumnProjection(
			int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return "->";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		if (left.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Collections.emptySet();
		if (right.stream()
				.noneMatch(t -> t.isStringType() && t.isNumericType() && t.asNumericType().isIntegral()
						&& t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))
						&& t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference())))
			return Collections.emptySet();
		if (right.stream()
				.anyMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference())))
			return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES));
	}
}
