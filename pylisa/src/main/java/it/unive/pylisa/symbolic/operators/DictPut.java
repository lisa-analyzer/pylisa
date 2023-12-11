package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class DictPut implements TernaryOperator {

	public static final DictPut INSTANCE = new DictPut();

	private DictPut() {
	}

	@Override
	public String toString() {
		return "put";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> middle, Set<Type> right) {
		if (left.stream().noneMatch(t -> t.toString().equals(LibrarySpecificationProvider.DICT)))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.DICT));
	}

}