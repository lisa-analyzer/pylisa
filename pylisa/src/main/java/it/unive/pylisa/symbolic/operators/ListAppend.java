package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
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
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.toString().equals(LibrarySpecificationProvider.LIST)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.LIST));
	}

}
