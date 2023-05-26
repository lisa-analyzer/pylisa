package it.unive.pylisa.symbolic.operators;

import java.util.Collections;
import java.util.Set;
import java.util.function.Predicate;

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
		Predicate<Type> accepted = t -> t.isNumericType() || t.isNullType();
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) { 
			ReferenceType seriesreftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES).getReference();
			accepted = accepted.or(t -> t.equals(seriesreftype));
		}
		
		if (left.stream().noneMatch(accepted))
			return Collections.emptySet();
		if (middle.stream().noneMatch(accepted))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.isNumericType() || t.isNullType()))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.SLICE));
	}

}
