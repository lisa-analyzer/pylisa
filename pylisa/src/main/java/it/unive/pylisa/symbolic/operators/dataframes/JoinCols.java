package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class JoinCols implements BinaryOperator, DataframeOperator {

	private final int index;

	public JoinCols(int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		if (right.stream().noneMatch(
				t -> t.equals(df) || t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES))))
			return Collections.emptySet();
		return Collections.singleton(df);
	}

	@Override
	public String toString() {
		return "concat_cols->";
	}
}
