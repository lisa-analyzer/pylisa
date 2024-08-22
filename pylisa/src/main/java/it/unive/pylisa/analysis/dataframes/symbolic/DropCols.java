package it.unive.pylisa.analysis.dataframes.symbolic;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class DropCols implements BinaryOperator, DataframeOperator {

	private final int index;

	public DropCols(
			int index) {
		this.index = index;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return "drop_columns->";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.LIST))))
			return Collections.emptySet();
		return Collections.singleton(df);
	}
}
