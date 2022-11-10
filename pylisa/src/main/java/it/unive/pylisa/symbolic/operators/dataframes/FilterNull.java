package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class FilterNull implements UnaryOperator {

	public static enum Axis {
		ROWS, COLUMNS, TOP
	}

	private final Axis axis;

	public FilterNull(Axis axis) {
		this.axis = axis;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	public String toString() {
		return "filter_null:" + axis;
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (argument.stream().noneMatch(t -> t.equals(df)))
			return Collections.emptySet();
		return Collections.singleton(df);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		FilterNull other = (FilterNull) obj;
		if (axis != other.axis)
			return false;
		return true;
	}
}
