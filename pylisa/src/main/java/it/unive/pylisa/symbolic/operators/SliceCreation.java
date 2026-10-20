package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class SliceCreation implements TernaryOperator {

	public static final SliceCreation INSTANCE = new SliceCreation();

	private SliceCreation() {
	}

	@Override
	public String toString() {
		return "new_slice";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> middle,
			Set<Type> right) {
		// any value can be a bound of a slice: invalid bounds raise TypeError
		// when the slice is used (e.g. to index a list)
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.SLICE));
	}

}
