package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ProjectRows implements TernaryOperator {

	public static final ProjectRows INSTANCE = new ProjectRows();

	private ProjectRows() {
	}

	@Override
	public String toString() {
		return "head";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> middle, Set<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		if (middle.stream().noneMatch(Type::isNumericType))
			return Collections.emptySet();
		if (right.stream().noneMatch(Type::isNumericType))
			return Collections.emptySet();
		return Collections.singleton(df);
	}
}
