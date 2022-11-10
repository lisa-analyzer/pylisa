package it.unive.pylisa.symbolic.operators;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ListAppend implements BinaryOperator {

	public static final ListAppend INSTANCE = new ListAppend();

	private ListAppend() {
	}

	@Override
	public String toString() {
		return "append";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		if (left.stream().noneMatch(t -> t.toString().equals(LibrarySpecificationProvider.LIST)))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.LIST));
	}

}
