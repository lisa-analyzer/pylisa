package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class DictPut implements TernaryOperator {

	public static final DictPut INSTANCE = new DictPut();

	private DictPut() {
	}

	@Override
	public String toString() {
		return "put";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.toString().equals(LibrarySpecificationProvider.DICT)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.DICT));
	}

}