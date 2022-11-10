package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ColumnAccess implements BinaryOperator {

	public static final ColumnAccess INSTANCE = new ColumnAccess();

	private ColumnAccess() {
	}

	@Override
	public String toString() {
		return "->";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		if (left.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Collections.emptySet();
		if (right.stream()
				.noneMatch(t -> t.isStringType() && t.isNumericType() && t.asNumericType().isIntegral()
						&& t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))
						&& t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference())))
			return Collections.emptySet();
		if (right.stream().anyMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference())))
			return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF));
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES));
	}
}
