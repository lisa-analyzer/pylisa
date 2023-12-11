package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class SetAdd implements BinaryOperator {

	public static final SetAdd INSTANCE = new SetAdd();

	private SetAdd() {
	}

	@Override
	public String toString() {
		return "add";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		if (left.stream().noneMatch(t -> t.toString().equals(LibrarySpecificationProvider.SET)))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.SET));
	}

}