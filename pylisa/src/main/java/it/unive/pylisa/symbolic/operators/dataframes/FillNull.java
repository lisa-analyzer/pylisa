package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class FillNull implements BinaryOperator {

	public static enum Axis {
		ROWS, COLUMNS, TOP
	}

	private final Axis axis;

	public FillNull(Axis axis) {
		this.axis = axis;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	public String toString() {
		return "fill_null:" + axis;
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.noneMatch(t -> t.equals(df)))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(Type::isNumericType))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
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
		FillNull other = (FillNull) obj;
		if (axis != other.axis)
			return false;
		return true;
	}
}
