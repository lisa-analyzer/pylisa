package it.unive.pylisa.symbolic.operators;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class SliceCreation implements TernaryOperator {

	public static final SliceCreation INSTANCE = new SliceCreation();

	private SliceCreation() {
	}

	@Override
	public String toString() {
		return "new_slice";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> middle, Set<Type> right) {
		ReferenceType seriesref = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES).getReference();
		if (left.stream().noneMatch(t -> t.isNumericType() || t.isNullType() || t.equals(seriesref)))
			return Collections.emptySet();
		if (middle.stream().noneMatch(t -> t.isNumericType() || t.isNullType() || t.equals(seriesref)))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.isNumericType() || t.isNullType()))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.SLICE));
	}

}
