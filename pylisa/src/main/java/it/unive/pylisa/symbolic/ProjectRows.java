package it.unive.pylisa.symbolic;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.libraries.pandas.PyDataframeType;

public class ProjectRows implements TernaryOperator {

	public static final ProjectRows INSTANCE = new ProjectRows();

	private ProjectRows() {
	}

	@Override
	public String toString() {
		return "head";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> middle, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PyDataframeType.REFERENCE)))
			return Caches.types().mkEmptySet();
		if (middle.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyDataframeType.INSTANCE);
	}
}
